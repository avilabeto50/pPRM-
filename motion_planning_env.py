import pygame
import random
import math
import heapq

# Constants
SEED =  random.randint(0, 1000000)  # For reproducibility
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800
ROBOT_COUNT = 60
ROBOT_RADIUS = 15
OBSTACLE_COUNT = 5
GOAL_RADIUS = 20
N_SAMPLES = 500
GAMMA_PRM = 500.0
D = 2

# Colors
WHITE = (255, 255, 255)
GRAY = (128, 128, 128)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

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

def segments_intersect(p1, q1, p2, q2):
    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0
        return 1 if val > 0 else 2

    def on_segment(p, q, r):
        if min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[1] <= max(p[1], r[1]):
            return True
        return False

    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and on_segment(p1, p2, q1):
        return True
    if o2 == 0 and on_segment(p1, q2, q1):
        return True
    if o3 == 0 and on_segment(p2, p1, q2):
        return True
    if o4 == 0 and on_segment(p2, q1, q2):
        return True

    return False

def line_intersects_polygon(p1, p2, polygon_vertices):
    for i in range(len(polygon_vertices)):
        q1 = polygon_vertices[i]
        q2 = polygon_vertices[(i + 1) % len(polygon_vertices)]
        if segments_intersect(p1, p2, q1, q2):
            return True
    return False

class Robot:
    def __init__(self, position, radius, color, speed= 100):
        self.position = position
        self.radius = radius
        self.color = color
        self.speed = speed  # pixels per second
        self.path = None  # will be set after PRM computation
        self.current_waypoint_index = 0
        self.reached_goal = False

    def set_path(self, path):
        """Set the path for this robot to follow."""
        self.path = path
        self.current_waypoint_index = 0
        self.reached_goal = False

    def update(self, dt):
        """Update robot position along the path. dt is elapsed time in seconds."""
        if not self.path or self.reached_goal or self.current_waypoint_index >= len(self.path) - 1:
            self.reached_goal = True
            return

        # Current position (robot's actual position) and target waypoint
        current = self.position
        target = self.path[self.current_waypoint_index + 1]

        # Distance to target
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        distance_to_target = math.hypot(dx, dy)

        # Distance we can move this frame
        distance_to_move = self.speed * dt

        if distance_to_move >= distance_to_target:
            # We've reached or passed the target waypoint
            self.position = target
            self.current_waypoint_index += 1
        else:
            # Move towards target
            if distance_to_target > 0:
                ratio = distance_to_move / distance_to_target
                self.position = (
                    current[0] + dx * ratio,
                    current[1] + dy * ratio
                )

    def get_target_waypoint(self):
        """Get the current target waypoint."""
        if self.path and self.current_waypoint_index < len(self.path) - 1:
            return self.path[self.current_waypoint_index + 1]
        return None

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

    def get_inflated_vertices(self, inflation_factor=0.1):
        if not self.vertices:
            return []
        # Calculate centroid
        cx = sum(x for x, y in self.vertices) / len(self.vertices)
        cy = sum(y for x, y in self.vertices) / len(self.vertices)
        inflated = []
        for x, y in self.vertices:
            # Vector from center to vertex
            dx = x - cx
            dy = y - cy
            # Scale by (1 + inflation_factor)
            scale = 1 + inflation_factor
            nx = cx + dx * scale
            ny = cy + dy * scale
            inflated.append((nx, ny))
        return inflated

class Environment:
    def __init__(self, robots, obstacles, goal):
        self.robots = robots
        self.obstacles = obstacles
        self.goal = goal  # (position, radius, color)

    def render(self, screen):
        screen.fill(WHITE)
        for obs in self.obstacles:
            pygame.draw.polygon(screen, obs.color, obs.vertices)
        
        # Draw paths and waypoints for each robot
        for i, robot in enumerate(self.robots):
            if robot.path:
                # Draw path as blue lines
                for j in range(len(robot.path) - 1):
                    pygame.draw.line(screen, BLUE, robot.path[j], robot.path[j + 1], 2)
                
                # Draw waypoints as small gray circles
                for waypoint in robot.path:
                    pygame.draw.circle(screen, GRAY, (int(waypoint[0]), int(waypoint[1])), 3)
                
                # Draw target waypoint as a yellow circle
                target = robot.get_target_waypoint()
                if target:
                    pygame.draw.circle(screen, (255, 255, 0), (int(target[0]), int(target[1])), 5)
        
        # Draw robots on top
        for robot in self.robots:
            pygame.draw.circle(screen, robot.color, (int(robot.position[0]), int(robot.position[1])), robot.radius)
        
        # Draw goal
        pygame.draw.circle(screen, self.goal[2], (int(self.goal[0][0]), int(self.goal[0][1])), self.goal[1])

class PRM:
    def __init__(self, obstacles, start, goal, n_samples, gamma, d, inflation_factor=0.1):
        self.obstacles = obstacles
        self.start = start
        self.goal = goal
        self.n_samples = n_samples
        self.gamma = gamma
        self.d = d
        self.inflation_factor = inflation_factor
        self.vertices = [start, goal]
        self.edges = {}  # dict of lists, v: [(u, dist)]
        self.build_graph()

    def sample_free(self):
        while True:
            x = random.uniform(0, WINDOW_WIDTH)
            y = random.uniform(0, WINDOW_HEIGHT)
            point = (x, y)
            if self.is_free(point):
                return point

    def is_free(self, point):
        for obs in self.obstacles:
            inflated = obs.get_inflated_vertices(self.inflation_factor)
            if point_in_polygon(point, inflated):
                return False
        return True

    def near(self, v, r):
        near_points = []
        for u in self.vertices:
            if u != v and dist(v, u) <= r:
                near_points.append(u)
        return near_points

    def collision_free(self, v, u):
        for obs in self.obstacles:
            inflated = obs.get_inflated_vertices(self.inflation_factor)
            if line_intersects_polygon(v, u, inflated):
                return False
        return True

    def build_graph(self):
        # Sample points
        for _ in range(self.n_samples):
            sample = self.sample_free()
            self.vertices.append(sample)
        
        # For each v, find near, connect if free
        for v in self.vertices:
            r = self.gamma * (math.log(len(self.vertices)) / len(self.vertices)) ** (1 / self.d)
            U = self.near(v, r)
            for u in U:
                if self.collision_free(v, u):
                    if v not in self.edges:
                        self.edges[v] = []
                    if u not in self.edges:
                        self.edges[u] = []
                    dist_vu = dist(v, u)
                    self.edges[v].append((u, dist_vu))
                    self.edges[u].append((v, dist_vu))  # undirected

    def dijkstra(self):
        # Find shortest path from start to goal
        queue = [(0, self.start, [])]  # (dist, node, path)
        visited = set()
        dists = {self.start: 0}
        prev = {self.start: None}
        while queue:
            current_dist, current, path = heapq.heappop(queue)
            if current in visited:
                continue
            visited.add(current)
            path = path + [current]
            if current == self.goal:
                return path
            if current not in self.edges:
                continue
            for neighbor, weight in self.edges[current]:
                if neighbor in visited:
                    continue
                new_dist = current_dist + weight
                if neighbor not in dists or new_dist < dists[neighbor]:
                    dists[neighbor] = new_dist
                    prev[neighbor] = current
                    heapq.heappush(queue, (new_dist, neighbor, path))
        return None  # no path

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

    # Build PRM and find paths for each robot
    for robot in env.robots:
        prm = PRM(env.obstacles, robot.position, env.goal[0], N_SAMPLES, GAMMA_PRM, D)
        path = prm.dijkstra()
        if path:
            robot.set_path(path)
            print(f"Path found for robot at {robot.position}: length {len(path)}")
        else:
            print(f"No path found for robot at {robot.position}")

    # Set up game loop
    clock = pygame.time.Clock()
    FPS = 60

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0  # Convert milliseconds to seconds
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Update robot positions
        for robot in env.robots:
            robot.update(dt)
        
        env.render(screen)
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()