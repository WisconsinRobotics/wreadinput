from typing import cast

import rospy

from .shape import DeviceShape
from .wreadinput import WReadInput

def main(node_name: str, shape: DeviceShape):
    rospy.init_node(node_name)

    controller_name = rospy.get_param('~name', node_name)
    namespace = rospy.get_param('~namespace', None)
    wready_server_ns = rospy.get_param('~wready_ns', '/wready')

    with WReadInput.find(controller_name, shape, wready_server_ns) as wreadinput:
        wreadinput.spin(namespace)
