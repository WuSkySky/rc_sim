import math


_QUATERNION_NORM_SQUARED_EPSILON = 1.0e-12


def normalize_quaternion(quaternion):
    norm_squared = sum(component * component for component in quaternion)
    if (
        not math.isfinite(norm_squared)
        or norm_squared <= _QUATERNION_NORM_SQUARED_EPSILON
    ):
        return None

    inverse_norm = 1.0 / math.sqrt(norm_squared)
    return tuple(component * inverse_norm for component in quaternion)


def multiply_quaternions(first, second):
    first_x, first_y, first_z, first_w = first
    second_x, second_y, second_z, second_w = second
    return (
        first_w * second_x
        + first_x * second_w
        + first_y * second_z
        - first_z * second_y,
        first_w * second_y
        - first_x * second_z
        + first_y * second_w
        + first_z * second_x,
        first_w * second_z
        + first_x * second_y
        - first_y * second_x
        + first_z * second_w,
        first_w * second_w
        - first_x * second_x
        - first_y * second_y
        - first_z * second_z,
    )


def rotate_vector(quaternion, vector):
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    conjugate = (
        -quaternion[0],
        -quaternion[1],
        -quaternion[2],
        quaternion[3],
    )
    rotated = multiply_quaternions(
        multiply_quaternions(quaternion, vector_quaternion),
        conjugate,
    )
    return rotated[:3]


def invert_transform(translation, rotation):
    inverse_rotation = (
        -rotation[0],
        -rotation[1],
        -rotation[2],
        rotation[3],
    )
    inverse_translation = rotate_vector(
        inverse_rotation,
        tuple(-component for component in translation),
    )
    return inverse_translation, inverse_rotation


def compose_transforms(first_translation, first_rotation,
                       second_translation, second_rotation):
    rotated_translation = rotate_vector(first_rotation, second_translation)
    translation = tuple(
        first_translation[index] + rotated_translation[index]
        for index in range(3)
    )
    rotation = normalize_quaternion(
        multiply_quaternions(first_rotation, second_rotation)
    )
    return translation, rotation


def quaternion_from_rpy(roll, pitch, yaw):
    half_roll = roll / 2.0
    half_pitch = pitch / 2.0
    half_yaw = yaw / 2.0

    sin_roll = math.sin(half_roll)
    cos_roll = math.cos(half_roll)
    sin_pitch = math.sin(half_pitch)
    cos_pitch = math.cos(half_pitch)
    sin_yaw = math.sin(half_yaw)
    cos_yaw = math.cos(half_yaw)

    return (
        sin_roll * cos_pitch * cos_yaw
        - cos_roll * sin_pitch * sin_yaw,
        cos_roll * sin_pitch * cos_yaw
        + sin_roll * cos_pitch * sin_yaw,
        cos_roll * cos_pitch * sin_yaw
        - sin_roll * sin_pitch * cos_yaw,
        cos_roll * cos_pitch * cos_yaw
        + sin_roll * sin_pitch * sin_yaw,
    )


def is_finite(values):
    return all(math.isfinite(component) for component in values)
