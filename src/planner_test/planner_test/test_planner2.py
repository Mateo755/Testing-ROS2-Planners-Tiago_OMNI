#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import time
import csv
import sys
from pathlib import Path
import os

class PlannerTester(Node):
    def __init__(self, planner_name, goals_file):
        super().__init__('planner_tester')
        self.planner_name = planner_name
        self.goals_file = goals_file
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.results = []

    def load_goals(self):
        goals = []
        with open(self.goals_file, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) != 2:
                    continue
                x, y = map(float, parts)
                goals.append((x, y))
        return goals

    def send_and_measure(self, x, y):
        # Zbuduj startową pozycję (zakładamy mapowe (0.0, 0.0))
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = 0.0
        start.pose.position.y = 0.0
        start.pose.orientation.w = 1.0

        # Zbuduj pozycję celu
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        # Pobierz plan PRZED ruchem
        path = self.navigator.getPath(start, goal)

        # Oblicz długość ścieżki
        path_length = 0.0
        for i in range(1, len(path.poses)):
            x1, y1 = path.poses[i - 1].pose.position.x, path.poses[i - 1].pose.position.y
            x2, y2 = path.poses[i].pose.position.x, path.poses[i].pose.position.y
            path_length += ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

        # Zmierz czas ruchu
        self.get_logger().info(f'Sending goal: ({x}, {y})')
        start_time = time.time()
        self.navigator.goToPose(goal)

        while not self.navigator.isTaskComplete():
            time.sleep(0.1)

        plan_time = time.time() - start_time
        self.results.append({
            "planner": self.planner_name,
            "goal_x": x,
            "goal_y": y,
            "plan_time_sec": round(plan_time, 3),
            "path_length_m": round(path_length, 3)
        })

    def save_to_csv(self):
        filename = f'results_{self.planner_name}.csv'
        filepath = Path('/home/mateusz/ROS2/tiago_public_ws/data') / filename

        with open(filepath, mode='w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.results[0].keys())
            writer.writeheader()
            writer.writerows(self.results)

        self.get_logger().info(f'Wyniki zapisane do: {filepath}')

# def main():
#     if len(sys.argv) < 3:
#         print("Użycie: ros2 run <twoja_paczka> test_planner.py <planner_name> <goals.txt>")
#         return

#     planner_name = sys.argv[1]
#     goals_file = sys.argv[2]

#     rclpy.init()
#     tester = PlannerTester(planner_name, goals_file)
#     goals = tester.load_goals()

#     for x, y in goals:
#         tester.send_and_measure(x, y)

#     tester.save_to_csv()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

def main():
    rclpy.init()
    planner_name = "navfn"  # wpisz nazwę planera
    tester = PlannerTester(planner_name, goals_file=None)

    # punkt docelowy (x, y) wpisany na sztywno:
    goal_x = -11.46
    goal_y = 2.43

    tester.send_and_measure(goal_x, goal_y)
    tester.save_to_csv()
    rclpy.shutdown()
