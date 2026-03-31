import json
from typing import Any

import numpy as np
from geometry_msgs.msg import Twist


def sanitize_observation(obs: Any, expected_dim: int, fill_value: float = 0.0) -> np.ndarray:
    arr = np.asarray(obs, dtype=np.float32).reshape(-1)
    if arr.size < expected_dim:
        arr = np.pad(arr, (0, expected_dim - arr.size), mode='constant', constant_values=fill_value)
    elif arr.size > expected_dim:
        arr = arr[:expected_dim]
    return np.nan_to_num(arr, nan=fill_value, posinf=fill_value, neginf=fill_value)


def normalized_action_to_twist(
    action: Any,
    max_vx: float,
    max_vy: float,
    max_vz: float,
    max_yaw_rate: float,
) -> Twist:
    arr = np.asarray(action, dtype=np.float32).reshape(-1)
    if arr.size != 4:
        raise ValueError(f'action must have 4 elements, got {arr.size}')
    arr = np.clip(arr, -1.0, 1.0)

    msg = Twist()
    msg.linear.x = float(arr[0] * max_vx)
    msg.linear.y = float(arr[1] * max_vy)
    msg.linear.z = float(arr[2] * max_vz)
    msg.angular.z = float(arr[3] * max_yaw_rate)
    return msg


def zero_twist() -> Twist:
    return Twist()


def parse_info_json(raw: str) -> dict[str, Any]:
    if not raw:
        return {}
    try:
        data = json.loads(raw)
        return data if isinstance(data, dict) else {}
    except json.JSONDecodeError:
        return {'raw_info': raw}
