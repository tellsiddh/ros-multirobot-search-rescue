import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
import random
from matplotlib.animation import PillowWriter  # For GIFs

# Define the environment size and the obstacles
grid_size = (20, 20)
robot_positions = {(0, 0), (grid_size[0] - 1, 0), (0, grid_size[1] - 1), (grid_size[0] - 1, grid_size[1] - 1)}

# Generate random obstacles
obstacles = set()
while len(obstacles) < 50:
    new_obstacle = (random.randint(0, grid_size[0] - 1), random.randint(0, grid_size[1] - 1))
    if new_obstacle not in robot_positions and new_obstacle not in obstacles:
        obstacles.add(new_obstacle)

# Initialize the exploration grid with zeros and mark robot positions as explored
explored_map = np.zeros(grid_size)
for pos in robot_positions:
    explored_map[pos] = 1


# Define the Robot class with added color and path attributes
class Robot:
    def __init__(self, start_pos, explored_map, color):
        self.position = start_pos
        self.explored_map = explored_map.copy()
        self.frontier = None
        self.color = color
        self.path = [start_pos]  # Initialize the path with the start position
        self.total_distance = 0  # Initialize total distance traveled

    def detect_frontiers(self, global_explored_map):
        # Find edges of the explored areas
        frontiers = []
        for x in range(global_explored_map.shape[0]):
            for y in range(global_explored_map.shape[1]):
                if global_explored_map[x, y] == 0 and (x, y) not in obstacles:
                    # Check if the neighboring cell is explored (value 1)
                    if ((x > 0 and global_explored_map[x - 1, y] == 1) or
                        (x < global_explored_map.shape[0] - 1 and global_explored_map[x + 1, y] == 1) or
                        (y > 0 and global_explored_map[x, y - 1] == 1) or
                        (y < global_explored_map.shape[1] - 1 and global_explored_map[x, y + 1] == 1)):
                        frontiers.append((x, y))
        print(f"Detected frontiers: {frontiers}")
        return frontiers


    def select_frontier(self, frontiers):
        # Select the closest unassigned frontier
        closest = None
        min_dist = np.inf
        for f in frontiers:
            dist = np.linalg.norm(np.array(self.position) - np.array(f))
            if dist < min_dist:
                min_dist = dist
                closest = f
        return closest

    def move_towards_target(self, target):
        if target is None:
            return False
        x_diff = target[0] - self.position[0]
        y_diff = target[1] - self.position[1]

        # Prioritize movement in the direction of the larger difference
        if abs(x_diff) > abs(y_diff):
            # Move horizontally
            new_x = self.position[0] + np.clip(x_diff, -1, 1)
            new_position = (new_x, self.position[1])
        else:
            # Move vertically
            new_y = self.position[1] + np.clip(y_diff, -1, 1)
            new_position = (self.position[0], new_y)

        if new_position not in obstacles and new_position not in [r.position for r in robots]:
            self.position = new_position
            return True
        return False


    def random_walk(self):
        # Attempt to move to a random neighboring cell
        moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        random.shuffle(moves)
        for move in moves:
            new_position = (self.position[0] + move[0], self.position[1] + move[1])
            if (0 <= new_position[0] < grid_size[0] and
                    0 <= new_position[1] < grid_size[1] and
                    new_position not in obstacles and
                    new_position not in [r.position for r in robots]):
                self.position = new_position
                return True
        return False

    def move(self):
        if not self.frontier:
            self.frontier = self.select_frontier(available_frontiers)
        if self.frontier and self.move_towards_target(self.frontier):
            if self.position == self.frontier:
                self.explored_map[self.frontier] = 1
                self.frontier = None
        elif self.position == self.frontier:
            self.explored_map[self.frontier] = 1
            self.frontier = None
        else:
            # If the robot could not move towards a frontier, try random walk
            if not self.random_walk():
                print(f"Robot at {self.position} is stuck and cannot random walk.")
        
        self.path.append(self.position)  # Record the new position in the path


# Function to visualize the map, obstacles, robots, and their paths
def visualize(robots, global_map, ax):
    ax.clear()
    ax.imshow(global_map.T, cmap='Greys', origin='lower', extent=(0, grid_size[0], 0, grid_size[1]))
    for obs in obstacles:
        ax.add_patch(Rectangle(obs, 1, 1, color='red'))
    for robot in robots:
        # Draw robot position
        ax.plot(robot.position[0] + 0.5, robot.position[1] + 0.5, 'o', color=robot.color)
        # Draw robot path
        if len(robot.path) > 1:
            path_x, path_y = zip(*robot.path)
            ax.plot(path_x, path_y, color=robot.color, linewidth=2)

# Initialize the robots with different colors
colors = ['blue', 'green', 'orange', 'purple']  # Define as many colors as robots
robots = [Robot(pos, explored_map, color) for pos, color in zip(robot_positions, colors)]


# Initialize the figure
fig, ax = plt.subplots()

# Define the animation update function
def update(frame):
    global explored_map, available_frontiers
    # Detect all frontiers from the current map
    available_frontiers = robots[0].detect_frontiers(explored_map)
    
    # Check if there are no more frontiers
    if not available_frontiers:
        print("No more frontiers detected. Stopping simulation.")
        ani.event_source.stop()  # Stop the animation
        return

    # Each robot makes a move
    for robot in robots:
        robot.move()
        explored_map = np.maximum(explored_map, robot.explored_map)
    
    # Visualize the map and robots
    visualize(robots, explored_map, ax)

# Create the animation
ani = FuncAnimation(fig, update, frames=200, interval=100)

# Save the animation
filename = "robot_exploration.gif"
ani.save(filename, writer=PillowWriter(fps=20))

plt.show()