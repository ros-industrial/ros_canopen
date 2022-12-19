from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition
from std_srvs.srv import Trigger
from canopen_interfaces.srv import CORead, COWrite, COReadID, COWriteID


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
