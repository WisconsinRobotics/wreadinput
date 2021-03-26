from typing import Any, List

def get_capability_codes(cap_arr: List[Any]) -> List[int]:
    if len(cap_arr) == 0 or isinstance(cap_arr[0], int):
        return cap_arr
    return [cap[0] for cap in cap_arr]
