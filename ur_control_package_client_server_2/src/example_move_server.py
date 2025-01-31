import time
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import yaml  # Import the yaml module
import threading

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from custom_action_interfaces.action import Manipulation


class ServerLvl2(Node):
    def __init__(self):
        print('Starting server_lvl_2')
        super().__init__('server_lvl_2')
        self._action_server = ActionServer(self, Manipulation, 'manipulation', self.execute_callback)
        global common_data
        self.common_data = common_data

    def execute_callback(self, goal_handle):
        self.common_data['execution_start_flag'].set()
        self.get_logger().info('Executing goal...')

        # OMOGUĆITI DA SE FEEDBACK MOŽE TRAŽITI IZ KLIJENTA NA CALLBACK
        feedback_msg = Manipulation.Feedback()
        feedback_msg.current_state = ['Current status: ' + self.common_data['status']]
        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        self.common_data['execution_finished_flag'].wait()
        result = Manipulation.Result()
        return result


class JTCClient(Node):
    """Client for the jtc (joint trajectory controller)."""
    def __init__(self):
        print('Starting jtc_client')
        super().__init__("jtc_client")
        global common_data
        self.common_data = common_data
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self._action_client.wait_for_server()

    def execute_callback(self):
        self.common_data['execution_start_flag'].wait()
        self.common_data['status'] = 'started'
        self.parse_trajectories()
        self.i = 0
        self.execute_next_trajectory()

    def parse_trajectories(self):
        self.goals = {}
        with open('src/trajectories.yaml', 'r') as file:
            data = yaml.safe_load(file)
            for traj_name, points in data['trajectories'].items():
                goal = JointTrajectory()
                goal.joint_names = self.joints
                for pt in points:
                    point = JointTrajectoryPoint()
                    point.positions = pt["positions"]
                    point.velocities = pt["velocities"]
                    point.time_from_start = Duration(**pt["time_from_start"])  # Unpack the dictionary
                    goal.points.append(point)

                self.goals[traj_name] = goal

    def execute_next_trajectory(self):
        if self.i >= len(self.goals):  # If last goal finished
            self.get_logger().info("Done with all trajectories")
            self.common_data['execution_finished_flag'].set()
            self.common_data['status'] = 'finished'
            raise SystemExit
        traj_name = list(self.goals)[self.i]
        self.common_data['status'] = 'Executing trajectory ' + traj_name
        self.i += 1
        if traj_name:
            self.execute_trajectory(traj_name)

    def execute_trajectory(self, traj_name):
        self.get_logger().info(f"Executing trajectory {traj_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.goals[traj_name]

        goal.goal_time_tolerance = Duration(sec=0, nanosec=1000)

        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            raise RuntimeError("Goal rejected")

        self.get_logger().debug("Goal accepted")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Done with result: {self.error_code_to_str(result.error_code)}")

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            time.sleep(1)
            self.execute_next_trajectory()
        else:
            raise RuntimeError("Executing trajectory failed")

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"


def spin_action_server(node):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()


def main(args=None):
    rclpy.init(args=args)
    global common_data 
    common_data = {'status': '',
                   'execution_start_flag': threading.Event(),
                   'execution_finished_flag': threading.Event()}
    
    action_server_node = ServerLvl2()
    action_client_node = JTCClient()

    # Start the action server in a new thread
    server_thread = threading.Thread(target=spin_action_server, args=(action_server_node,))
    server_thread.start()

    # Execute the action client logic here
    action_client_node.execute_callback()

    rclpy.spin(action_client_node)


if __name__ == "__main__":
    main()
