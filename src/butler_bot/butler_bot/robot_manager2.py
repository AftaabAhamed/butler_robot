from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
 
def main():
    rclpy.init()
    nav = BasicNavigator()
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = nav.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.2
    init_pose.pose.position.y = 0.0
    init_pose.pose.position.z = 0.15
    init_pose.pose.orientation.x = 0.0
    init_pose.pose.orientation.y = 0.0
    init_pose.pose.orientation.z = 0.0
    init_pose.pose.orientation.w = 1.0

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 3.998
    goal_pose.pose.position.y = -1.434
    goal_pose.pose.position.z = 0.150
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    # ...

    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

    # ...

    path = nav.getPath(init_pose, goal_pose)
    smoothed_path = nav.smoothPath(path)

    # ...

    nav.goToPose(goal_pose)
    while not nav.isTaskComplete():

        # feedback = nav.getFeedback()
        # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
        #     nav.cancelTask()
        str_in = input('Enter "cancel" to cancel the goal: ')
        if str_in == 'cancel':
            nav.cancelTask()
            break

    # ...

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    
    nav.lifecycleShutdown()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
  main()