import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from refbox_msgs.msg import GamePlay, NavigationRoutesData
import time
import threading

class NavigationChallenge(Node):
    def __init__(self):
        super().__init__('navigation_challenge_node')

        self.initial_variables()

        self.create_subscription(GamePlay, "/refbox/game_play", self.commandGamePlayCallback, 10)
        self.create_subscription(NavigationRoutesData, "/refbox/navigation_routes", self.commandNavigationRouteCallback, 10)

        self.timer = self.create_timer(0.1, self.update_callback)

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.get_logger().info("Navigation Challenge : Setup subscriber on /refbox/game_play [refbox_msgs/GamePlay]")
        self.get_logger().info("Navigation Challenge : Setup subscriber on /refbox/navigation_routes [refbox_msgs/NavigationRoutesData]")

    def initial_variables(self):
        self.game_play_msg = GamePlay()
        self.navigation_routes_msg = NavigationRoutesData()
        self.route_thread = None
        self.route_running = False
        self.lock = threading.Lock()

    def commandGamePlayCallback(self, msg):
        with self.lock:
            self.game_play_msg = msg

    def commandNavigationRouteCallback(self, msg):
        with self.lock:
            self.navigation_routes_msg = msg

    def make_pose(self, x, y, yaw, frame="map"):
        import math
        import tf_transformations

        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = rclpy.clock.Clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

    def moveToPoint(self, x, y, yaw, frame="map"):
        goal = self.make_pose(x, y, yaw, frame)

        # These are blocking and MUST be outside callbacks -> now OK (thread)
        self.navigator.goToPose(goal)

        while rclpy.ok():
            # check game status (no spin here!)
            with self.lock:
                game_on = bool(self.game_play_msg.status)

            if not game_on:
                self.get_logger().warn("Game state FALSE -> cancel")
                self.navigator.cancelTask()
                return False  # stop route immediately

            if self.navigator.isTaskComplete():
                break

            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} m")

            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal reached successfully!")
            time.sleep(2)
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Goal canceled")
            return False
        else:
            self.get_logger().error("Goal failed")
            return False
    
    def update_callback(self):
        # only start once
        if self.route_running:
            return

        with self.lock:
            has_routes = len(self.navigation_routes_msg.routes) != 0
            game_on = bool(self.game_play_msg.status)

        if game_on and has_routes:
            self.route_running = True
            self.get_logger().info("start")
            self.route_thread = threading.Thread(target=self.run_route, daemon=True)
            self.route_thread.start()

    def run_route(self):
        # copy route data once (avoid race)
        with self.lock:
            route = self.navigation_routes_msg.routes[0].route

        # waypoint 1
        x1 = route[0].position.x - 0.5
        y1 = route[0].position.y - 0.5
        self.moveToPoint(x1, y1, 0.0)
        time.sleep(5)

        # waypoint 2
        x2 = route[1].position.x - 0.5
        y2 = route[1].position.y - 0.5
        self.moveToPoint(x2, y2, 0.0)
        time.sleep(5)

def main(args=None):
    rclpy.init(args=args)

    challenge_node = NavigationChallenge()

    navigator_node = challenge_node.navigator

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)

    executor.add_node(challenge_node)
    executor.add_node(navigator_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        challenge_node.destroy_node()
        navigator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()