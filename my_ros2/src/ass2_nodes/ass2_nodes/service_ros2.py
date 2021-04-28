# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ass2_services.srv import RandPos

import rclpy
import random
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('service_ros2')
        self.srv = self.create_service(RandPos, 'random_pos', self.randNumbers_callback)

    def randNumbers_callback(self, request, response):
        x = random.uniform(-8.0, 8.0)
        y = random.uniform(-8.0, 8.0)
        response.x = x
        response.y = y
        self.get_logger().info('Done: %d, %d\n' %
        (x, y))
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
