# Copyright 2026 taneesh
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Node to monitor and log robot navigation metrics against goal."""

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node


class NavMonitor(Node):
    """Monitor node tracking current goal, remaining distance, and state."""

    def __init__(self):
        """Initialize subscriptions, variables, and timer."""
        super().__init__('nav_monitor')

        # Subscriptions
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.plan_sub = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10
        )
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            10
        )
        self.feedback_sub = self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.feedback_callback,
            10
        )
        self.planner_status_sub = self.create_subscription(
            GoalStatusArray,
            '/compute_path_to_pose/_action/status',
            self.planner_status_callback,
            10
        )

        # State variables
        self.goal_x = None
        self.goal_y = None
        self.remaining_distance = None

        # State values: IDLE, NAVIGATING, REPLANNING, SUCCEEDED, FAILED
        self.nav_status = 'IDLE'
        self.active_goal_id = None
        self.planner_active = False

        # Timer for logging dashboard (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Navigation Monitor Node started.')

    def goal_callback(self, msg: PoseStamped):
        """Record the target navigation goal coordinates from /goal_pose."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    def plan_callback(self, msg: Path):
        """Record goal coordinates from the end of the planned path."""
        if msg.poses:
            last_pose = msg.poses[-1]
            self.goal_x = last_pose.pose.position.x
            self.goal_y = last_pose.pose.position.y

    def feedback_callback(self, msg: NavigateToPose.Impl.FeedbackMessage):
        """Update the remaining distance to the goal from action feedback."""
        self.remaining_distance = msg.feedback.distance_remaining

    def planner_status_callback(self, msg: GoalStatusArray):
        """Check if the planner server is actively computing a path."""
        active = False
        for goal_status in msg.status_list:
            if goal_status.status in [1, 2]:  # ACCEPTED or EXECUTING
                active = True
                break
        self.planner_active = active

    def status_callback(self, msg: GoalStatusArray):
        """Update navigation state by tracking GoalStatusArray transitions."""
        active_goal = None
        for goal_status in msg.status_list:
            if goal_status.status in [1, 2]:
                active_goal = goal_status
                break

        if active_goal is not None:
            goal_id = bytes(active_goal.goal_info.goal_id.uuid).hex()
            if self.active_goal_id != goal_id:
                # Transition to NAVIGATING (New Goal)
                self.active_goal_id = goal_id
                self.nav_status = 'NAVIGATING'
                self.remaining_distance = None
            else:
                # If active goal matches, check if planner is running
                if self.planner_active:
                    self.nav_status = 'REPLANNING'
                else:
                    self.nav_status = 'NAVIGATING'
        else:
            # No active goals. Check if we were navigating and just finished
            if self.active_goal_id is not None:
                completed_status = None
                for goal_status in msg.status_list:
                    goal_id = bytes(goal_status.goal_info.goal_id.uuid).hex()
                    if goal_id == self.active_goal_id:
                        completed_status = goal_status.status
                        break

                if completed_status is not None:
                    if completed_status == 4:  # SUCCEEDED
                        self.nav_status = 'SUCCEEDED'
                    elif completed_status in [5, 6]:  # CANCELED or ABORTED
                        self.nav_status = 'FAILED'
                else:
                    self.nav_status = 'IDLE'

                self.active_goal_id = None
                self.remaining_distance = 0.0

    def timer_callback(self):
        """Print log report matching requested formatting in assessment."""
        goal_str = 'None'
        if self.goal_x is not None and self.goal_y is not None:
            goal_str = f'({self.goal_x:.2f}, {self.goal_y:.2f})'

        dist_str = 'N/A'
        if self.remaining_distance is not None:
            dist_str = f'{self.remaining_distance:.2f} m'

        print('\n' + '='*40)
        print(f'Current Goal: {goal_str}')
        print(f'Remaining Distance: {dist_str}')
        print(f'Status: {self.nav_status}')
        print('='*40)


def main(args=None):
    """Initialize ROS 2 context and start spin loop."""
    rclpy.init(args=args)
    node = NavMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
