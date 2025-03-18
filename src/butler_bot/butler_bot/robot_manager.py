import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


from butler_interfaces.action import TableOrder
from butler_interfaces.srv import CancelOrder
from butler_interfaces.srv import TableConf
from butler_interfaces.srv import KitchenConf

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration

import asyncio


class ButlerActionServer(Node):

    def __init__(self):
        super().__init__('butler_action_server')
        self._action_server = ActionServer(
            self,
            TableOrder,
            'take_order',
            self.execute_callback)
        
        self._cancel_order = self.create_service(CancelOrder, 'cancel_order', self.cancel_order_callback)

        self._table_conf = self.create_client(TableConf, 'table_conf')
        self._kitchen_conf = self.create_client(KitchenConf, 'kitchen_conf')

        while not self._table_conf.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self._kitchen_conf.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.recvd_order = [] # List of orders received
        self.onboard_order = [] # List of orders onboard the robot
        self.active_order = [] # List of orders to deliver recieved orders without cancelled orders

        self.current_destination = "home"

        self.nav = BasicNavigator()

        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.nav.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.2
        init_pose.pose.position.y = 0.0
        init_pose.pose.position.z = 0.15
        init_pose.pose.orientation.x = 0.0
        init_pose.pose.orientation.y = 0.0
        init_pose.pose.orientation.z = 0.0
        init_pose.pose.orientation.w = 1.0

        k = PoseStamped()
        k.header.frame_id = 'map'
        k.header.stamp = self.nav.get_clock().now().to_msg()
        k.pose.position.x = 0.015
        k.pose.position.y = -2.000
        k.pose.position.z = 0.150
        k.pose.orientation.x = 0.000
        k.pose.orientation.y = 0.000
        k.pose.orientation.z = 0.0
        k.pose.orientation.w = 1.0

        t1 = PoseStamped()
        t1.header.frame_id = 'map'
        t1.header.stamp = self.nav.get_clock().now().to_msg()
        t1.pose.position.x = 0.755
        t1.pose.position.y = -4.589
        t1.pose.position.z = 0.150
        t1.pose.orientation.x = 0.000
        t1.pose.orientation.y = 0.000
        t1.pose.orientation.z = 0.0
        t1.pose.orientation.w = 1.0

        t2 = PoseStamped()
        t2.header.frame_id = 'map'
        t2.header.stamp = self.nav.get_clock().now().to_msg()
        t2.pose.position.x = 2.822
        t2.pose.position.y = -4.569
        t2.pose.position.z = 0.150
        t2.pose.orientation.x = 0.000
        t2.pose.orientation.y = 0.000
        t2.pose.orientation.z = 0.0
        t2.pose.orientation.w = 1.0

        t3 = PoseStamped()
        t3.header.frame_id = 'map'
        t3.header.stamp = self.nav.get_clock().now().to_msg()
        t3.pose.position.x = 3.343
        t3.pose.position.y = -0.649
        t3.pose.position.z = 0.150
        t3.pose.orientation.x = 0.000
        t3.pose.orientation.y = 0.000
        t3.pose.orientation.z = 0.0
        t3.pose.orientation.w = 1.0


        self.pose_dict = {
            "home": init_pose,
            "table1": t1,
            "table2": t2,
            "table3": t3,
            "kitchen": k,
        }

        
        self.nav.setInitialPose(init_pose)
        self.nav.waitUntilNav2Active()
    
    def cancel_order_callback(self, request, response):
        self.get_logger().info('Cancel order request recieved')
        order_to_cancel = request.tableid
        if order_to_cancel in self.active_order:
            self.active_order.remove(order_to_cancel)
            response.status = True
        else:
            response.status = False
        return response

    def get_table_conf(self, req):
        request = TableConf.Request()
        request.tableid = req 
        future = self._table_conf.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
        else:
            response = False
        return response
    
    def get_kitchen_conf(self, req):
        request = KitchenConf.Request()
        request.order = req
        future = self._kitchen_conf.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
        else:
            response = False
        return response

    def go_to_destination(self, destination):

        self.current_destination = destination
        self.nav.goToPose(self.pose_dict[destination])
        self.get_logger().info(f"Going to {destination}")
        

        while not self.nav.isTaskComplete():

            if self.current_destination not in self.active_order and self.current_destination != "home" and self.current_destination != "kitchen":
                self.get_logger().info(f"Order {self.current_destination} has been cancelled")
                # self.nav.cancelTask()
                return False

        return True

    def execute_callback(self, goal_handle):

        feedback = TableOrder.Feedback()

        self.get_logger().info('Executing goal...')

        self.recvd_order = goal_handle.request.order
        
        self._logger.info(f"Received order: {self.recvd_order}")
        reached = self.go_to_destination("kitchen")
        if reached:
            resp = self.get_kitchen_conf(self.recvd_order)

            if resp is None:
                feedback.status = "Order could not prepared by kitchen"
                goal_handle.publish_feedback(feedback)
                self.get_logger().info("Order could not be prepared by kitchen")
                self.go_to_destination("home")
                goal_handle.abort()
                return

            self.onboard_order = resp.ready

        self.active_order = self.onboard_order.copy()



        while self.active_order!=[]:

            destination = self.active_order[0]

            reached = self.go_to_destination(destination)         

            if reached:
                resp = self.get_table_conf(destination)
                if resp.status:
                    feedback.status = f"Order {destination} delivered"
                    goal_handle.publish_feedback(feedback)
                    self.get_logger().info(f"Order {destination} delivered")
                    self.active_order.pop(0)
                    self.onboard_order.pop(0)
                else:
                    self.get_logger().info(f"Order {destination} could not be delivered")
                    self.active_order.pop(0)

            else:

                feedback.status = "order cancelled before delivery"
                goal_handle.publish_feedback(feedback)
                continue
            
        if self.onboard_order != []:
            feedback.status = "Returning to kitchen with remaining orders"
            goal_handle.publish_feedback(feedback)
            reached = self.go_to_destination("kitchen")
            if reached:
                resp = self.get_kitchen_conf(self.onboard_order)
                if not resp:
                    feedback.status = "leftover order could not be returned to kitchen and returning to home"
                    goal_handle.publish_feedback(feedback)
                    self.get_logger().info("Order could not be returned to kitchen")

        resp = self.go_to_destination("home")

        goal_handle.succeed()

        result = TableOrder.Result()
        result.status = self.onboard_order
        return result


def main(args=None):
    rclpy.init(args=args)

    butler_action_server = ButlerActionServer()

    rclpy.spin(butler_action_server)


if __name__ == '__main__':
    main()