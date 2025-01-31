import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_action_interfaces.action import Manipulation


class ClientLvl2(Node):
    def __init__(self):
        print('Starting client_lvl_2')
        super().__init__('client_lvl_2')
        self._action_client = ActionClient(self, Manipulation, 'manipulation')

    def send_goal(self): #, order):
        goal_msg = Manipulation.Goal()
        #goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Task done with result: {0}'.format(result.result_message))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_state))


def main(args=None):
    rclpy.init(args=args)
    action_client = ClientLvl2()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()