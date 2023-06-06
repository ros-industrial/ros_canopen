#    Copyright 2022 Christoph Hellmann Santos
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition
from std_srvs.srv import Trigger
from canopen_interfaces.srv import CORead, COWrite, COReadID, COWriteID
from canopen_interfaces.msg import COData


class SimpleTestNode(Node):
    def __init__(self, name="test_node"):
        super().__init__(name)

    def checkRpdoTpdo(self, node_name, index: int, subindex: int, type: int, data: int) -> bool:
        publisher = self.create_publisher(COData, "/" + node_name + "/tpdo", 10)
        subscriber = self.create_subscription(
            COData, "/" + node_name + "/rpdo", self.rpdo_callback, 10
        )
        target = COData()
        target.index = index
        target.subindex = subindex
        target.type = type
        target.data = data
        publisher.publish(target)
        print("Published tpdo")
        # Wait for topic to be published
        self.rpdo_data = None
        while not (self.rpdo_data):
            print("Waiting for subscriber to connect. ")
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.rpdo_data.data == data:
            result = True
        else:
            result = False

        self.destroy_publisher(publisher)
        self.destroy_subscription(subscriber)

        return result

    def rpdo_callback(self, msg):
        print("RPDO Callback")
        print(msg)
        self.rpdo_data = msg


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTestNode()

    result = node.checkRpdoTpdo("proxy_device_1", index=0x4000, subindex=0, type=32, data=999)
    print("RESULT: " + str(result))

    result = node.checkRpdoTpdo("proxy_device_2", index=0x4000, subindex=0, type=32, data=999)
    print("RESULT: " + str(result))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
