import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from ros2_pkg.action import Navigate


class NavigateAtionClient(Node):
    def __init__(self):
        super().__init__("action_client_node")
        self._action_client = ActionClient (self, Navigate, 'navigate')

    def send_goal(self, x, y, z):
        goal_msg = Navigate.Goal()
        goal_msg.goal_point.x = float(x)
        goal_msg.goal_point.y = float(y)
        goal_msg.goal_point.z = float(z)

        self._action_client.wait_for_server()
        self.send_goal_future = self._action_client.send_goal_async(goal_msg, self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_responce_callback)

    def callback_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print("Received Feedback: " + str(feedback.distance_to_point))    

    def goal_responce_callback(self, future):
        goal_handle = future.result()

        if goal_handle.accepted == False:
            print("Goal handle Rejected")
            return None
        
        print("Accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print("Result " + str(result.elapsed_time) + "seconds")
        self.rclpy.shutdown()



def main (args= None):
    rclpy.init()
    action_client_node = NavigateAtionClient()
    print("Action Client is Running...")

    try:
        x = input("Enter a X Coordinate: ")
        y = input("Enter a Y Coordinate: ")
        z = input("Enter a Z Coordinate: ")

        action_client_node.send_goal(x, y, z)
        rclpy.spin(action_client_node)
        
    except KeyboardInterrupt:
        print("Terminating Node...")
        action_client_node.destroy_node()
    
if __name__ == '__main__':
    main()