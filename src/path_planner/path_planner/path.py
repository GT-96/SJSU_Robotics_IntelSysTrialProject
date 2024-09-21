import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import heapq
import json

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Subscriptions
        self.map_subscription = self.create_subscription(String, '/map', self.map_callback, 10)
        self.start_subscription = self.create_subscription(String, '/start', self.start_callback, 10)
        self.goal_subscription = self.create_subscription(String, '/goal', self.goal_callback, 10)

        # Publisher for the path
        self.path_publisher = self.create_publisher(String, '/path', 10)

        self.grid = None
        self.start = None
        self.goal = None

    def map_callback(self, msg):
        self.grid = [list(map(int, row)) for row in msg.data.split('\n')]

    def start_callback(self, msg):
        self.start = tuple(map(int, msg.data.split(',')))

    def goal_callback(self, msg):
        self.goal = tuple(map(int, msg.data.split(',')))

        # Once we have all data, start A* algorithm
        if self.grid and self.start and self.goal:
            self.a_star()

    def a_star(self):
        start, goal = self.start, self.goal
        width, height = len(self.grid[0]), len(self.grid)

        def heuristic(a, b):
            # Manhattan distance
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        # Initialize with start position
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while open_set:
            # Get the node with the lowest f_score from open set
            _, current = heapq.heappop(open_set)

            if current == goal:
                self.reconstruct_path(came_from, current)
                return

            neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Up, Down, Left, Right
            for x, y in neighbors:
                neighbor = (current[0] + x, current[1] + y)
                # Check if the neighbor is within the bounds of the grid
                if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height:
                    if self.grid[neighbor[1]][neighbor[0]] == 1:  # Skip walls
                        continue
                    # Get cost to reach neighbor
                    tentative_g_score = g_score[current] + 1
                    # check if neighbors been visited before or cost is lower
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        # update best path to neighbor, alogn with g and f score and append to heap
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # If we exit the loop and never reach the goal, we didn't find a path
        self.get_logger().info("No path found")
        return


    def reconstruct_path(self, came_from, current):
        # New list, starting from the goal
        total_path = [current]
        
        while current in came_from:
            # Move to the node that led to the current node and append
            current = came_from[current]
            total_path.append(current)

        total_path.reverse()

        self.publish_path(total_path)

    # Publish the entire path as 2D array
    def publish_path(self, path):
        # Convert path into a 2D array
        path_array = [[x, y] for (x, y) in path]

        # Convert the path to JSON
        msg = String()
        msg.data = json.dumps(path_array)
        self.path_publisher.publish(msg)

        self.get_logger().info(f'Published Path: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
