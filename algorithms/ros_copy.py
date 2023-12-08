import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
import random
from matplotlib.animation import PillowWriter  # For GIFs

# Define the environment size and the obstacles
grid_size = (20, 20)
center_pos = (grid_size[0] // 2, grid_size[1] // 2)
obstacles = set()
while len(obstacles) < 50:
    new_obstacle = (random.randint(0, grid_size[0] - 1), random.randint(0, grid_size[1] - 1))
    if new_obstacle != center_pos:
        obstacles.add(new_obstacle)

# Initialize the exploration grid with zeros
explored_map = np.zeros(grid_size)

class Robot:
    def __init__(self, start_pos, explored_map, color):
        self.position = start_pos
        self.explored_map = explored_map.copy()
        self.color = color
        self.path = [start_pos]
        self.ready_to_explore = False  # Indicates whether the robot is ready to explore

    def calculate_potential(self, other_robots):
        potential = 0
        for other in other_robots:
            if other != self:
                distance = np.linalg.norm(np.array(self.position) - np.array(other.position))
                potential += 1 / distance if distance != 0 else np.inf

        for obs in obstacles:
            distance = np.linalg.norm(np.array(self.position) - np.array(obs))
            potential += 1 / distance if distance != 0 else np.inf

        return potential

    def move_towards_min_potential(self, other_robots):
        min_potential = self.calculate_potential(other_robots)
        best_move = None

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                new_position = (self.position[0] + dx, self.position[1] + dy)
                if (0 <= new_position[0] < grid_size[0] and
                        0 <= new_position[1] < grid_size[1] and
                        new_position not in obstacles and
                        new_position not in [r.position for r in other_robots]):
                    self.position = new_position
                    potential = self.calculate_potential(other_robots)
                    if potential < min_potential:
                        min_potential = potential
                        best_move = new_position

        if best_move:
            self.position = best_move
            self.path.append(self.position)

    def at_local_minimum(self, other_robots):
        current_potential = self.calculate_potential(other_robots)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                new_position = (self.position[0] + dx, self.position[1] + dy)
                if (0 <= new_position[0] < grid_size[0] and
                        0 <= new_position[1] < grid_size[1] and
                        new_position not in obstacles and
                        new_position not in [r.position for r in other_robots]):
                    self.position = new_position
                    potential = self.calculate_potential(other_robots)
                    if potential < current_potential:
                        return False
        return True

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
        print(f"Detected frontiers: {frontiers}")  # This will print the detected frontiers
        return frontiers

    def calculate_potential(self, other_robots):
        potential = 0
        # Repulsion from other robots
        for other in other_robots:
            if other != self:
                distance = np.linalg.norm(np.array(self.position) - np.array(other.position))
                potential += 1 / distance  # Increase potential with decreasing distance

        # Repulsion from obstacles
        for obs in obstacles:
            distance = np.linalg.norm(np.array(self.position) - np.array(obs))
            potential += 1 / distance

        return potential


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

    def move(self, other_robots):
        min_potential = self.calculate_potential(other_robots)
        best_move = None

        # Check potential for each neighboring cell
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                new_position = (self.position[0] + dx, self.position[1] + dy)
                if new_position in obstacles or new_position in [r.position for r in other_robots]:
                    continue
                self.position = new_position
                potential = self.calculate_potential(other_robots)
                if potential < min_potential:
                    min_potential = potential
                    best_move = new_position

        # Move to the position with the lowest potential
        if best_move:
            self.position = best_move



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
center_pos = (grid_size[0] // 2, grid_size[1] // 2)
robots = [Robot(center_pos, explored_map, color) for color in colors]


# Initialize the figure
fig, ax = plt.subplots()

def update(frame):
    global explored_map
    for robot in robots:
        if robot.ready_to_explore:
            robot.move()
        else:
            robot.move_towards_min_potential(robots)
            if robot.at_local_minimum(robots):
                robot.ready_to_explore = True

        explored_map = np.maximum(explored_map, robot.explored_map)

    visualize(robots, explored_map, ax)

ani = FuncAnimation(fig, update, frames=200, interval=100)
ani.save("robot_exploration.gif", writer=PillowWriter(fps=20))
plt.show()
