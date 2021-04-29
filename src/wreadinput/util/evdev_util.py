from typing import Any, List

def get_capability_codes(cap_arr: List[Any]) -> List[int]:
    """Extracts event codes from a capability list.

    In particular, the capability list for absolute axes
    includes some additional data about axis properties.
    This method will strip them so that event codes can be
    handled more uniformly.

    Parameters
    ----------
    cap_arr : List[Any]
        The capability list as provided by evdev.
    
    Returns
    -------
    List[int]
        A list of event codes corresponding to the device
        capabilities.
    """
    if len(cap_arr) == 0 or isinstance(cap_arr[0], int):
        return cap_arr
    return [cap[0] for cap in cap_arr]
