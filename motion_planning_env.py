import pygame
import random
import math

# Constants
SEED = 42
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800
ROBOT_COUNT = 5
ROBOT_RADIUS = 15
OBSTACLE_COUNT = 5
GOAL_RADIUS = 20

# Colors
WHITE = (255, 255, 255)
GRAY = (128, 128, 128)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# Helper functions
def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def point_in_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def line_intersects_circle(p1, p2, center, radius):
    d = (p2[0] - p1[0], p2[1] - p1[1])
    f = (center[0] - p1[0], center[1] - p1[1])
    a = d[0]**2 + d[1]**2
    b = 2 * (f[0] * d[0] + f[1] * d[1])
    c = f[0]**2 + f[1]**2 - radius**2
    discriminant = b**2 - 4 * a * c
    if discriminant < 0:
        return False
    discriminant = math.sqrt(discriminant)
    t1 = (-b - discriminant) / (2 * a)
    t2 = (-b + discriminant) / (2 * a)
    if 0 <= t1 <= 1 or 0 <= t2 <= 1:
        return True
    return False

def check_collision(circle_center, circle_radius, polygon_vertices):
    # Check if any vertex is inside circle
    for v in polygon_vertices:
        if dist(circle_center, v) <= circle_radius:
            return True
    # Check if circle center is inside polygon
    if point_in_polygon(circle_center, polygon_vertices):
        return True
    # Check if any edge intersects circle
    for i in range(len(polygon_vertices)):
        p1 = polygon_vertices[i]
        p2 = polygon_vertices[(i + 1) % len(polygon_vertices)]
        if line_intersects_circle(p1, p2, circle_center, circle_radius):
            return True
    return False

class Robot:
    def __init__(self, position, radius, color):
        self.position = position
        self.radius = radius
        self.color = color

class Obstacle:
    def __init__(self, vertices, color):
        self.vertices = vertices
        self.color = color
        # Calculate bounding circle for collision approximation
        if vertices:
            center_x = sum(x for x, y in vertices) / len(vertices)
            center_y = sum(y for x, y in vertices) / len(vertices)
            self.bounding_center = (center_x, center_y)
            self.bounding_radius = max(dist(self.bounding_center, v) for v in vertices)
        else:
            self.bounding_center = (0, 0)
            self.bounding_radius = 0

class Environment:
    def __init__(self, robots, obstacles, goal):
        self.robots = robots
        self.obstacles = obstacles
        self.goal = goal  # (position, radius, color)

    def render(self, screen):
        screen.fill(WHITE)
        for obs in self.obstacles:
            pygame.draw.polygon(screen, obs.color, obs.vertices)
        for robot in self.robots:
            pygame.draw.circle(screen, robot.color, (int(robot.position[0]), int(robot.position[1])), robot.radius)
        pygame.draw.circle(screen, self.goal[2], (int(self.goal[0][0]), int(self.goal[0][1])), self.goal[1])

def generate_obstacles():
    obstacles = []
    for _ in range(OBSTACLE_COUNT):
        while True:
            num_sides = random.randint(3, 6)
            center_x = random.randint(50, WINDOW_WIDTH - 50)
            center_y = random.randint(50, WINDOW_HEIGHT - 50)
            radius = random.randint(20, 50)
            vertices = []
            for i in range(num_sides):
                angle = 2 * math.pi * i / num_sides + random.uniform(0, 2 * math.pi / num_sides)
                x = center_x + radius * math.cos(angle)
                y = center_y + radius * math.sin(angle)
                vertices.append((x, y))
            obs = Obstacle(vertices, GRAY)
            # Check no overlap with existing obstacles
            overlap = False
            for existing in obstacles:
                if dist(obs.bounding_center, existing.bounding_center) <= obs.bounding_radius + existing.bounding_radius:
                    overlap = True
                    break
            if not overlap:
                obstacles.append(obs)
                break
    return obstacles

def generate_robots(obstacles):
    robots = []
    for _ in range(ROBOT_COUNT):
        while True:
            x = random.randint(ROBOT_RADIUS, 400 - ROBOT_RADIUS)
            y = random.randint(ROBOT_RADIUS, WINDOW_HEIGHT - ROBOT_RADIUS)
            position = (x, y)
            robot = Robot(position, ROBOT_RADIUS, RED)
            # Check no overlap with obstacles or other robots
            overlap = False
            for obs in obstacles:
                if dist(position, obs.bounding_center) <= ROBOT_RADIUS + obs.bounding_radius:
                    overlap = True
                    break
            if not overlap:
                for existing in robots:
                    if dist(position, existing.position) <= ROBOT_RADIUS + existing.radius:
                        overlap = True
                        break
            if not overlap:
                robots.append(robot)
                break
    return robots

def generate_goal(obstacles, robots):
    while True:
        x = random.randint(400 + GOAL_RADIUS, WINDOW_WIDTH - GOAL_RADIUS)
        y = random.randint(GOAL_RADIUS, WINDOW_HEIGHT - GOAL_RADIUS)
        position = (x, y)
        goal = (position, GOAL_RADIUS, GREEN)
        # Check no overlap with obstacles or robots
        overlap = False
        for obs in obstacles:
            if dist(position, obs.bounding_center) <= GOAL_RADIUS + obs.bounding_radius:
                overlap = True
                break
        if not overlap:
            for robot in robots:
                if dist(position, robot.position) <= GOAL_RADIUS + robot.radius:
                    overlap = True
                    break
        if not overlap:
            return goal

def main():
    random.seed(SEED)
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Motion Planning Environment")

    obstacles = generate_obstacles()
    robots = generate_robots(obstacles)
    goal = generate_goal(obstacles, robots)

    env = Environment(robots, obstacles, goal)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        env.render(screen)
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()