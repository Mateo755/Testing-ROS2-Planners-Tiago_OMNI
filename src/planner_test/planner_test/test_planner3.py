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

        #self.get_logger().info(f"Costmap size: {len(data)}, max: {max(data)}, min: {min(data)}")


        total_cost = 0
        valid_points = 0

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
                        total_cost += cell_cost
                        valid_points += 1

        if valid_points == 0:
            return None

        avg_cost = total_cost / valid_points
        return round(avg_cost, 2)


    
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

        try:
            global_costmap = self.navigator.getGlobalCostmap()
            global_cost = self.compute_path_cost(path, global_costmap)
        except Exception as e:
            self.get_logger().warn(f"Global costmap unavailable: {e}")
            global_cost = None

        # Brak local cost map - scan topic not working
        # try:
        #     local_costmap = self.navigator.getLocalCostmap()
        #     local_cost = self.compute_path_cost(path, local_costmap)
        # except Exception as e:
        #     self.get_logger().warn(f"Local costmap unavailable: {e}")
        #     local_cost = None

        self.results.append({
            "planner": self.planner_name,
            "goal_x": x,
            "goal_y": y,
            "planning_time_sec": round(planning_duration, 4),
            "path_length_m": round(path_length, 4),
            "path_cost_global": global_cost
            #"path_cost_local": local_cost
        })


    def save_to_csv(self):
        filename = f'results_{self.planner_name}.csv'
        filepath = Path('/home/mateusz/ROS2/omni_base_public_ws/data') / filename

        with open(filepath, mode='w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.results[0].keys())
            writer.writeheader()
            writer.writerows(self.results)

        self.get_logger().info(f'Wyniki zapisane do: {filepath}')


def main():
    rclpy.init()
    planner_name = "navfn"  # wpisz nazwę planera
    tester = PlannerTester(planner_name, goals_file=None)

    # punkt docelowy (x, y) wpisany na sztywno:
    # goal_x = -11.46
    # goal_y = 2.43

    goal_x = 2.22
    goal_y = 6.36

    tester.send_and_measure(goal_x, goal_y)
    tester.save_to_csv()
    rclpy.shutdown()
