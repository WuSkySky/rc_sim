import re, html, json
raw = open('.research_tmp2/btn.html', encoding='utf-8', errors='replace').read()
m = re.search(r'"mainEntity":{"@type":"Question".*?"suggestedAnswer":([.*?])}', raw, flags=re.S)
if m:
    data = json.loads(m.group(1))
    for a in data:
        if isinstance(a, dict) and a.get('text'):
            print('*', a['text'][:800].replace('\n', ' '))
            print()
else:
    t = re.sub(r'<[^>]+>', '\n', raw)
    lines = [l.strip() for l in t.splitlines() if l.strip()]
    for l in lines:
        if re.search(r'pull|button|GPIO12|low|level', l, re.I):
            print(l[:180])
