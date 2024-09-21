import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import json

# Visualization parameters
TILE_SIZE = 10
GRID_COLOR = (200, 200, 200)
WALL_COLOR = (0, 0, 0)
OPEN_COLOR = (255, 255, 255)
START_COLOR = (255, 255, 0)  # Yellow
GOAL_COLOR = (0, 255, 0)     # Green
PATH_COLOR = (255, 0, 0)     # Red

class Viz(Node):
    def __init__(self):
        super().__init__('viz')

        # Subscriptions
        self.create_subscription(String, '/map', self.map_callback, 10)
        self.create_subscription(String, '/start', self.start_callback, 10)
        self.create_subscription(String, '/goal', self.goal_callback, 10)
        self.create_subscription(String, '/path', self.path_callback, 10)

        # Pygame setup
        pygame.init()
        self.screen = None

        self.grid = None
        self.start = None
        self.goal = None
        self.path = []
        self.path_animation_index = 0  # To track animation progress
        self.animation_in_progress = False  # Flag to control animation state
        self.path_complete = False  # Flag to track when we have the full path

    def map_callback(self, msg):
        self.grid = [list(map(int, row)) for row in msg.data.split('\n')]
        self.render()

    def start_callback(self, msg):
        self.start = tuple(map(int, msg.data.split(',')))
        self.render()

    def goal_callback(self, msg):
        self.goal = tuple(map(int, msg.data.split(',')))
        self.render()

    def path_callback(self, msg):
        # Parse the JSON string into a 2D list of coordinates
        new_path = json.loads(msg.data)
        
        # Check if the new path contains both the start and goal
        if self.start and self.goal and self.start in new_path and self.goal in new_path:
            self.path = new_path
            self.path_complete = True  # Mark the path as complete
        else:
            self.path = new_path  # Update the path as it is not yet complete

        # Start animating the new path only if we are not in the middle of an animation
        if not self.animation_in_progress:
            self.path_animation_index = 0  # Reset animation index only once a new path is fully received
            self.animation_in_progress = True  # Start the animation

    def animate_path(self):
        if self.animation_in_progress:
            # Animate until we finish rendering the whole path
            if self.path_animation_index < len(self.path):
                self.render(animate=True)
                self.path_animation_index += 1
            else:
                # Once we reach the end, restart the animation
                self.path_animation_index = 0

    def render(self, animate=False):
        if self.grid is None:
            return

        height = len(self.grid)
        width = len(self.grid[0])

        if self.screen is None:
            self.screen = pygame.display.set_mode((width * TILE_SIZE, height * TILE_SIZE))
            pygame.display.set_caption("Map Visualization")

        self.screen.fill(OPEN_COLOR)

        # Draw grid
        for y in range(height):
            for x in range(width):
                color = OPEN_COLOR if self.grid[y][x] == 0 else WALL_COLOR
                rect = pygame.Rect(x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
                pygame.draw.rect(self.screen, color, rect)
                pygame.draw.rect(self.screen, GRID_COLOR, rect, 1)

        # Draw animated path if the path is being animated
        if animate:
            for x, y in self.path[:self.path_animation_index]:
                rect = pygame.Rect(x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
                pygame.draw.rect(self.screen, PATH_COLOR, rect)

        # Draw start
        if self.start:
            rect = pygame.Rect(self.start[0] * TILE_SIZE, self.start[1] * TILE_SIZE, TILE_SIZE, TILE_SIZE)
            pygame.draw.rect(self.screen, START_COLOR, rect)

        # Draw goal
        if self.goal:
            rect = pygame.Rect(self.goal[0] * TILE_SIZE, self.goal[1] * TILE_SIZE, TILE_SIZE, TILE_SIZE)
            pygame.draw.rect(self.screen, GOAL_COLOR, rect)

        pygame.display.update()

def main(args=None):
    rclpy.init(args=args)
    node = Viz()

    # Main loop to handle both ROS2 events and Pygame events
    running = True
    while rclpy.ok() and running:
        rclpy.spin_once(node, timeout_sec=0.1)  # Non-blocking call to handle ROS2 events

        # Handle Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Handle path animation if it's in progress
        if node.animation_in_progress:
            node.animate_path()

        pygame.time.wait(50)  # Small delay to reduce CPU usage

    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()
