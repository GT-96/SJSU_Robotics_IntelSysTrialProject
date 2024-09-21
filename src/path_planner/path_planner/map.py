import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        self.publisher_ = self.create_publisher(String, '/map', 10)
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.publish_map)
        
        # Map grid parameters
        self.map_width = 100  # columns
        self.map_height = 100  # rows
        
        # Generate the map
        self.grid = self.generate_map()

    # Generates the 2D Grid
    def generate_map(self):
        # Initialize with all 1's
        grid = [[1 for _ in range(self.map_width)] for _ in range(self.map_height)]
        min_path_length = (self.map_width * self.map_height) // 8

        # Keep track of each paths endpoint to connect them later
        path_endpoints = []
        self.create_isolated_paths(grid, min_path_length, path_endpoints)
        # self.connect_paths(grid, path_endpoints)
        return grid
    
    # Generates N number of isolated Paths for the Map
    def create_isolated_paths(self, grid, min_path_length, path_endpoints):
        for _ in range(4):  # Create N paths
            path_length = random.randint(min_path_length, min_path_length + 50)

            # Pick Random Start Point
            start_x = random.randint(0, self.map_width - 1)
            start_y = random.randint(0, self.map_height - 1)
            grid[start_y][start_x] = 0
            x, y = start_x, start_y
            
            # Proceed in random directions, eating away at the walls for the specified path length
            for _ in range(path_length):
                direction = random.choice(['up', 'down', 'left', 'right'])
                if direction == 'up' and y > 0:
                    y -= 1
                elif direction == 'down' and y < self.map_height - 1:
                    y += 1
                elif direction == 'left' and x > 0:
                    x -= 1
                elif direction == 'right' and x < self.map_width - 1:
                    x += 1
                grid[y][x] = 0
            path_endpoints.append((x, y))

    # Connects each path to eachother, in case the goal and start get placed in different isolated paths
    def connect_paths(self, grid, path_endpoints):
        for i in range(len(path_endpoints) - 1):
            x1, y1 = path_endpoints[i]
            x2, y2 = path_endpoints[i + 1]
            self.connect_points(grid, x1, y1, x2, y2)

    # Connect two paths to eachother
    def connect_points(self, grid, x1, y1, x2, y2):
        while x1 != x2:
            if x1 < x2:
                x1 += 1
            else:
                x1 -= 1
            grid[y1][x1] = 0
        while y1 != y2:
            if y1 < y2:
                y1 += 1
            else:
                y1 -= 1
            grid[y1][x1] = 0

    def publish_map(self):
        map_str = '\n'.join(''.join(str(cell) for cell in row) for row in self.grid)
        msg = String()
        msg.data = map_str
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Map:\n{msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
