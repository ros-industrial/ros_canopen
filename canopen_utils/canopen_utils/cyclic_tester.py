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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from canopen_interfaces.srv import COTargetDouble


class DoubleTalker(Node):
    """Publish messages to a topic using two publishers at different rates."""

    def __init__(self):
        super().__init__("double_talker")

        self.i = 0
        self.cli = self.create_client(COTargetDouble, "trinamic_pd42/target")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.increment = 1000.0
        self.period = 0.1
        self.value = 0.0
        self.group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(self.period, self.timer_callback, callback_group=self.group)
        self.req = COTargetDouble.Request()

    def timer_callback(self):
        self.value = self.value + self.increment
        self.req.target = self.value
        req = self.cli.call(self.req)
        if req.success:
            self.get_logger().info("Success")
        else:
            self.get_logger().info("Failure")


def main(args=None):
    rclpy.init(args=args)
    try:
        talker = DoubleTalker()
        # MultiThreadedExecutor executes callbacks with a thread pool. If num_threads is not
        # specified then num_threads will be multiprocessing.cpu_count() if it is implemented.
        # Otherwise it will use a single thread. This executor will allow callbacks to happen in
        # parallel, however the MutuallyExclusiveCallbackGroup in DoubleTalker will only allow its
        # callbacks to be executed one at a time. The callbacks in Listener are free to execute in
        # parallel to the ones in DoubleTalker however.
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(talker)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            talker.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
