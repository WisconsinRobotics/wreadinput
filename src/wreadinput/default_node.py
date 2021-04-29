from typing import cast

import rospy

from .shape import DeviceShape
from .wreadinput import WReadInput

def main(node_name: str, shape: DeviceShape):
    """A simple node that publishes data from an input device.

    The device is specified by a human operator through a WReady task, in which they
    are asked to press any button on the desired device. See `WReadInput::find` for
    more details on this process.

    Various ROS params are accepted, which are listed in "Other Parameters" below.

    Parameters
    ----------
    node_name : str
        The name of the ROS node.
    shape : DeviceShape
        The shape of the device to publish data for.
    
    Other Parameters
    ----------------
    name : str, optional
        A human-readable name identifying the device. By default, uses the node name.
    namespace : str, optional
        The ROS namespace to publish device data to. By default, uses the node's
        private namespace.
    wready_ns : str, optional
        The ROS namespace for the WReady server to use for device binding. By
        default, uses `/wready`.
    """
    rospy.init_node(node_name)

    controller_name = rospy.get_param('~name', node_name)
    namespace = rospy.get_param('~namespace', None)
    wready_server_ns = rospy.get_param('~wready_ns', '/wready')

    with WReadInput.find(controller_name, shape, wready_server_ns) as wreadinput:
        wreadinput.spin(namespace)
