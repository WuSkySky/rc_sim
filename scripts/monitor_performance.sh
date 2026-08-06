#!/usr/bin/env bash
# ROS 2 / Jetson performance monitor for rc_sim.
# Safe default: only samples an already-running system. Use --launch to let
# the script start and clean up its own launch process groups.

set -o pipefail

duration=30
workspace=/home/jetson/workspaces/rc_sim
launch_mode=none
output_dir="/tmp/rc_sim_perf_$(date +%Y%m%d_%H%M%S)"
topics=(/r2/front_camera/image_raw /r2/left_camera/image_raw /r2/right_camera/image_raw)

usage() {
  cat <<'EOF'
Usage: monitor_performance.sh [options]

Options:
  --duration SEC       Sampling duration (default: 30)
  --workspace PATH     ROS 2 workspace (default: /home/jetson/workspaces/rc_sim)
  --launch MODE        none, real1, real2, or both (default: none)
  --output DIR         Output directory (default: /tmp/rc_sim_perf_TIMESTAMP)
  --topic TOPIC        Topic to measure with ros2 topic hz (repeatable)
  -h, --help           Show this help

Examples:
  bash scripts/monitor_performance.sh --duration 60
  bash scripts/monitor_performance.sh --launch both --duration 60
EOF
}

while (($#)); do
  case "$1" in
    --duration)
      [[ ${2:-} =~ ^[0-9]+$ ]] || { echo "--duration must be an integer" >&2; exit 2; }
      duration=$2; shift 2 ;;
    --workspace)
      [[ -n ${2:-} ]] || { echo "--workspace needs a path" >&2; exit 2; }
      workspace=$2; shift 2 ;;
    --launch)
      [[ ${2:-} == none || ${2:-} == real1 || ${2:-} == real2 || ${2:-} == both ]] || {
        echo "--launch must be none, real1, real2, or both" >&2; exit 2;
      }
      launch_mode=$2; shift 2 ;;
    --output)
      [[ -n ${2:-} ]] || { echo "--output needs a path" >&2; exit 2; }
      output_dir=$2; shift 2 ;;
    --topic)
      [[ -n ${2:-} ]] || { echo "--topic needs a topic name" >&2; exit 2; }
      topics+=("$2"); shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

mkdir -p "$output_dir"
launch_pids=()
tegrastats_pid=""

collect_descendants() {
  local parent=$1 child
  while read -r child; do
    [[ -z $child ]] && continue
    echo "$child"
    collect_descendants "$child"
  done < <(ps -eo pid=,ppid= | awk -v parent="$parent" '$2 == parent {print $1}')
}

cleanup() {
  if [[ -n $tegrastats_pid ]]; then
    kill "$tegrastats_pid" 2>/dev/null || true
    wait "$tegrastats_pid" 2>/dev/null || true
  fi
  local pid
  for pid in "${launch_pids[@]}"; do
    # ros2 launch can place executables in child process groups. Capture the
    # complete descendant tree before signaling it so Odin is not orphaned.
    local children
    children=$(collect_descendants "$pid")
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    [[ -z $children ]] || kill -INT $children 2>/dev/null || true
  done
  sleep 2
  for pid in "${launch_pids[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
    local children
    children=$(collect_descendants "$pid")
    [[ -z $children ]] || kill -TERM $children 2>/dev/null || true
  done
  sleep 1
  for pid in "${launch_pids[@]}"; do
    local children
    children=$(collect_descendants "$pid")
    [[ -z $children ]] || kill -KILL $children 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

command -v tegrastats >/dev/null || { echo "tegrastats not found; run this on Jetson" >&2; exit 1; }
command -v ps >/dev/null || { echo "ps not found" >&2; exit 1; }

# Do not use nounset before sourcing ROS: setup.bash expects unset variables.
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "Missing /opt/ros/humble/setup.bash" >&2; exit 1
fi
if [[ -f "$workspace/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "$workspace/install/setup.bash"
else
  echo "Warning: $workspace/install/setup.bash not found; ROS package commands may fail" >&2
fi

start_launch() {
  local name=$1
  echo "Starting bringup/${name}.launch.py (output: $output_dir/$name.log)"
  setsid ros2 launch bringup "${name}.launch.py" >"$output_dir/$name.log" 2>&1 < /dev/null &
  launch_pids+=("$!")
}

case "$launch_mode" in
  real1) start_launch real1 ;;
  real2) start_launch real2 ;;
  both) start_launch real1; sleep 3; start_launch real2 ;;
esac

echo "Sampling for ${duration}s; output directory: $output_dir"
printf 'timestamp pid ppid cpu_percent mem_percent rss_kb comm args\n' >"$output_dir/processes.tsv"
tegrastats --interval 1000 >"$output_dir/tegrastats.log" 2>&1 &
tegrastats_pid=$!

end_time=$((SECONDS + duration))
while ((SECONDS < end_time)); do
  timestamp=$(date +%s)
  ps -eo pid=,ppid=,pcpu=,pmem=,rss=,comm=,args= | \
    awk -v ts="$timestamp" -v self="$$" '
      $1 != self && $6 !~ /^(awk|timeout)$/ && $0 ~ /(ros2|kfs|mipi|odin|camera|odometry|serial|stage|chassis|gripper|led|host_sdk|pcd|cloud|overlay|control|point|traverse|lift|rotate|align|detect|postprocess)/ {
        printf "%s %s %s %s %s %s %s %s\n", ts,$1,$2,$3,$4,$5,$6,$7
      }' >>"$output_dir/processes.tsv"
  sleep 1
done

if command -v ros2 >/dev/null; then
  ROS2CLI_DISABLE_DAEMON=1 timeout 8s ros2 node list --no-daemon >"$output_dir/nodes.txt" 2>&1 || true
  for topic in "${topics[@]}"; do
    safe_name=${topic#/}; safe_name=${safe_name//\//_}
    timeout 6s ros2 topic hz "$topic" --window 20 \
      --qos-reliability best_effort --qos-durability volatile \
      >"$output_dir/hz_${safe_name}.txt" 2>&1 || true
  done
fi

echo
echo '=== Process summary (ps %CPU is one-core percentage) ==='
awk 'NR > 1 {cpu[$7]+=$4; n[$7]++; if ($4 > peak[$7]) peak[$7]=$4; rss[$7]+=$6}
     END {for (p in n) printf "%-28s avg_cpu=%6.1f%% peak=%6.1f%% avg_rss=%8.0f KB samples=%d\n", p,cpu[p]/n[p],peak[p],rss[p]/n[p],n[p]}' \
    "$output_dir/processes.tsv" | sort

echo
echo '=== tegrastats summary ==='
awk '
  match($0,/CPU \[[^]]+\]/) {cpu=$0; sub(/.*CPU \[/,"",cpu); sub(/\].*/,"",cpu); ncores=split(cpu,cpu_values,","); sum=0; for(i=1;i<=ncores;i++){v=cpu_values[i]; sub(/%.*/,"",v); sum+=v} core_sum+=sum; core_avg+=sum/ncores; core_n++}
  match($0,/RAM [0-9]+\/[0-9]+MB/) {ram=$0; sub(/.*RAM /,"",ram); sub(/MB.*/,"",ram); split(ram,a,"/"); ram_sum+=a[1]; ram_n++; if (ram_n==1||a[1]<ram_min)ram_min=a[1]; if(a[1]>ram_max)ram_max=a[1]}
  match($0,/GR3D_FREQ[^ ]*/) {v=$0; sub(/.*GR3D_FREQ /,"",v); sub(/%.*/,"",v); if(v ~ /^[0-9]+$/){gpu_sum+=v;gpu_n++}}
  match($0,/VDD_IN [0-9]+mW/) {v=$0; sub(/.*VDD_IN /,"",v); sub(/mW.*/,"",v); if(v ~ /^[0-9]+$/){p_sum+=v;p_n++}}
  END {if(core_n)printf "CPU: avg %.1f%% per core (%.2f core-equivalents)\n",core_avg/core_n,core_sum/core_n/100;
       if(ram_n)printf "RAM used: avg %.0f MB, range %d-%d MB\n",ram_sum/ram_n,ram_min,ram_max;
       if(gpu_n)printf "GR3D_FREQ: avg %.1f%%\n",gpu_sum/gpu_n;
       if(p_n)printf "VDD_IN: avg %.0f mW\n",p_sum/p_n;}' "$output_dir/tegrastats.log"

echo
echo "Raw logs: $output_dir"
