import rclpy
from rclpy.node import Node
import time
import csv
import math
import statistics
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from pathlib import Path

class PlannerTester(Node):
    def __init__(self, planner_type):
        super().__init__('planner_tester')
        self.navigator = BasicNavigator()
        self.planner_name = planner_type  
        self.results = []
        self.path_dir = 'data'

    def compute_path_length(self, path):
        length = 0.0
        poses = path.poses
        for i in range(1, len(poses)):
            x1, y1 = poses[i - 1].pose.position.x, poses[i - 1].pose.position.y
            x2, y2 = poses[i].pose.position.x, poses[i].pose.position.y
            length += ((x2 - x1)**2 + (y2 - y1)**2) ** 0.5
        return length

    def compute_path_cost(self, path, costmap):
        metadata = costmap.metadata
        resolution = metadata.resolution
        width = metadata.size_x
        height = metadata.size_y
        origin_x = metadata.origin.position.x
        origin_y = metadata.origin.position.y
        data = costmap.data

        costs = []

        for pose in path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y

            mx = int((x - origin_x) / resolution)
            my = int((y - origin_y) / resolution)

            if 0 <= mx < width and 0 <= my < height:
                index = my * width + mx
                if 0 <= index < len(data):
                    cell_cost = data[index]
                    if cell_cost >= 0:
                        costs.append(cell_cost)

        if not costs:
            return None, None, None, None

        avg = round(statistics.mean(costs), 2)
        min_c = min(costs)
        max_c = max(costs)
        std = round(statistics.stdev(costs), 2) if len(costs) > 1 else 0.0

        return avg, min_c, max_c, std

    def compute_path_turning_complexity(self, path):
        total_angle_change = 0.0
        poses = path.poses

        for i in range(2, len(poses)):
            x0, y0 = poses[i - 2].pose.position.x, poses[i - 2].pose.position.y
            x1, y1 = poses[i - 1].pose.position.x, poses[i - 1].pose.position.y
            x2, y2 = poses[i].pose.position.x, poses[i].pose.position.y

            dx1, dy1 = x1 - x0, y1 - y0
            dx2, dy2 = x2 - x1, y2 - y1

            angle1 = math.atan2(dy1, dx1)
            angle2 = math.atan2(dy2, dx2)
            angle_diff = abs(angle2 - angle1)

            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff

            total_angle_change += angle_diff

        return round(total_angle_change, 4)

    def save_path_to_csv(self, path, filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["x", "y"])
            for pose in path.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                writer.writerow([x, y])

    def send_and_measure(self, x, y):
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = 0.0
        start.pose.position.y = 0.0
        start.pose.orientation.w = 1.0

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        planning_start = time.time()
        path = self.navigator.getPath(start, goal)
        planning_duration = time.time() - planning_start

        if not path or len(path.poses) < 2:
            self.get_logger().warn(f"Plan failed or too short for goal ({x}, {y})")
            return

        path_length = self.compute_path_length(path)
        path_turning = self.compute_path_turning_complexity(path)

        try:
            global_costmap = self.navigator.getGlobalCostmap()
            avg_cost, min_cost, max_cost, std_cost = self.compute_path_cost(path, global_costmap)
            self.get_logger().info(f"Costmap size: {len(global_costmap.data)}, max: {max(global_costmap.data)}, min: {min(global_costmap.data)}")
        except Exception as e:
            self.get_logger().warn(f"Global costmap unavailable: {e}")
            avg_cost = min_cost = max_cost = std_cost = None

        self.results.append({
            "planner": self.planner_name,
            "goal_x": x,
            "goal_y": y,
            "planning_time_sec": round(planning_duration, 4),
            "path_length_m": round(path_length, 4),
            "path_cost_global": avg_cost,
            "cost_min": min_cost,
            "cost_max": max_cost,
            "cost_std": std_cost,
            "path_turning_complexity": path_turning
        })

        filename = f"path_{self.planner_name}_{x:.2f}_{y:.2f}.csv".replace(" ", "_")
        path_filename = Path(self.path_dir) / filename
        
        self.save_path_to_csv(path, path_filename)
        self.get_logger().info(f"Zapisano ścieżkę do: {path_filename}")

    
    
    def save_to_csv(self, x, y):
        filename = f"results_{self.planner_name}_{x:.2f}_{y:.2f}.csv"
        filepath = Path(self.path_dir) / filename
        
        with open(filepath, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.results[0].keys())
            writer.writeheader()
            for row in self.results:
                writer.writerow(row)
        self.get_logger().info(f"Wyniki zapisane do: {filepath}")


def main(args=None):
    rclpy.init(args=args)

    planner_name = 'navfn'

    tester = PlannerTester(planner_type=planner_name)
    tester.navigator.waitUntilNav2Active()

    # Przykładowe punkty testowe
    # x_goal = -11.46
    # y_goal = 2.43

    x_goal = 2.22
    y_goal = 6.36
    

    #tester.get_logger().info("Getting path...")
    tester.send_and_measure(x_goal, y_goal)

    tester.save_to_csv(x_goal,y_goal)
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
