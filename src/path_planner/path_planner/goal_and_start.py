import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import math

class GoalAndStart(Node):
    def __init__(self):
        super().__init__('goal_and_start')
        
        self.subscription = self.create_subscription(String, '/map', self.map_callback, 10)

        self.start_publisher = self.create_publisher(String, '/start', 10)
        self.goal_publisher = self.create_publisher(String, '/goal', 10)

        self.start = None
        self.goal = None

        self.timer_period = 2.0
        self.timer = self.create_timer(self.timer_period, self.publish_coordinates)

    def map_callback(self, msg):
        if self.start is None and self.goal is None:

            # Create the appropriate sized grid based on the message
            grid = [list(map(int, row)) for row in msg.data.split('\n')]
            height = len(grid)
            width = len(grid[0])

            # Calculate the minimum distance as 1/8th of the diagonal
            min_distance = math.sqrt(width**2 + height**2) / 8

            # Pick random start and goal positions (that are open spaces)
            self.start = self.random_open_tile(grid, width, height)
            self.goal = self.random_open_tile(grid, width, height)

            # Ensure the goal is at least 1/8th of the grid's diagonal away from the start
            while self.manhattan_distance(self.start, self.goal) < min_distance:
                self.goal = self.random_open_tile(grid, width, height)

            self.get_logger().info(f'Generated Start: {self.start}, Goal: {self.goal}')

    def random_open_tile(self, grid, width, height):
        while True:
            x = random.randint(0, width - 1)
            y = random.randint(0, height - 1)
            if grid[y][x] == 0:
                return x, y

    def manhattan_distance(self, start, goal):
        return abs(goal[0] - start[0]) + abs(goal[1] - start[1])

    def publish_coordinates(self):
        if self.start and self.goal:
            start_msg = String()
            goal_msg = String()

            # Format as x,y
            start_msg.data = f'{self.start[0]},{self.start[1]}'  
            goal_msg.data = f'{self.goal[0]},{self.goal[1]}'
            
            self.start_publisher.publish(start_msg)
            self.goal_publisher.publish(goal_msg)

            self.get_logger().info(f'Published Start: {start_msg.data}, Goal: {goal_msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalAndStart()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
