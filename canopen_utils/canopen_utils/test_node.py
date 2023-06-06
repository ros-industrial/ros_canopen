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
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition
from std_srvs.srv import Trigger
from canopen_interfaces.srv import CORead, COWrite, COReadID, COWriteID
from canopen_interfaces.msg import COData


class TestNode(Node):
    def __init__(self, name="test_node"):
        super().__init__(name)

    def checkTransition(self, node_name: str, state: int, tranisition: int) -> bool:
        get_state_client = self.create_client(GetState, node_name + "/get_state")
        change_state_client = self.create_client(ChangeState, node_name + "/change_state")
        if not get_state_client.wait_for_service(timeout_sec=3.0):
            get_state_client.destroy()
            change_state_client.destroy()
            return False
        if not change_state_client.wait_for_service(timeout_sec=3.0):
            get_state_client.destroy()
            change_state_client.destroy()
            return False
        req = GetState.Request()
        result = get_state_client.call(req)
        if result.current_state.id != state:
            return False

        req = ChangeState.Request()
        req.transition.id = tranisition
        result = change_state_client.call(req)
        if not result.success:
            get_state_client.destroy()
            change_state_client.destroy()
            return False
        get_state_client.destroy()
        change_state_client.destroy()
        return True

    def checkTrigger(self, node_name, service_name) -> bool:
        trigger_client = self.create_client(Trigger, "/" + node_name + "/" + service_name)
        if not trigger_client.wait_for_service(timeout_sec=3.0):
            return False
        req = Trigger.Request()
        result = trigger_client.call(req)
        trigger_client.destroy()
        if result.success:
            return True
        return False

    def checkSDORead(self, node_name, index: int, subindex: int, type: int, data: int) -> bool:
        client = self.create_client(CORead, "/" + node_name + "/sdo_read")
        if not client.wait_for_service(timeout_sec=3.0):
            return False
        req = CORead.Request()
        req.index = index
        req.subindex = subindex
        req.type = type
        result = client.call(req)
        client.destroy()
        if result.success and (data == result.data):
            return True
        return False

    def checkSDOWrite(self, node_name, index: int, subindex: int, type: int, data: int) -> bool:
        client = self.create_client(COWrite, "/" + node_name + "/sdo_write")
        if not client.wait_for_service(timeout_sec=3.0):
            return False
        req = COWrite.Request()
        req.index = index
        req.subindex = subindex
        req.data = data
        req.type = type
        result = client.call(req)
        client.destroy()
        if result.success:
            return True
        return False

    def checkSDOReadID(
        self, node_id: int, index: int, subindex: int, type: int, data: int
    ) -> bool:
        client = self.create_client(COReadID, "/master/sdo_read")
        if not client.wait_for_service(timeout_sec=3.0):
            return False
        req = COReadID.Request()
        req.index = index
        req.subindex = subindex
        req.type = type
        req.nodeid = node_id
        result = client.call(req)
        client.destroy()
        if result.success and (data == result.data):
            return True
        return False

    def checkSDOWriteID(
        self, node_id: int, index: int, subindex: int, type: int, data: int
    ) -> bool:
        client = self.create_client(COWriteID, "/master/sdo_write")
        if not client.wait_for_service(timeout_sec=3.0):
            return False
        req = COWriteID.Request()
        req.index = index
        req.subindex = subindex
        req.data = data
        req.type = type
        req.nodeid = node_id
        result = client.call(req)
        client.destroy()
        if result.success:
            return True
        return False

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
        print("Published tpdo to topic: " + "/" + node_name + "/tpdo")
        self.rpdo_data = None
        # Wait for topic to be published
        while not self.rpdo_data:
            rclpy.spin_once(self, timeout_sec=0.5)

        self.destroy_publisher(publisher)
        self.destroy_subscription(subscriber)

        if self.rpdo_data.data == data:
            return True
        return False

    def rpdo_callback(self, msg):
        print(msg)
        self.rpdo_data = msg
