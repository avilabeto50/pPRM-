import heapq
import math
import os
import random
import concurrent.futures
from queue import Queue
from dataclasses import dataclass
from typing import Optional, List, Tuple

import pygame
import torch

# Constants
SEED = 76
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800
ROBOT_COUNT = 20
ROBOT_RADIUS = 10
OBSTACLE_COUNT = 20
GOAL_RADIUS = 20
N_SAMPLES = 500
GAMMA_PRM = 500.0
D = 2
SAMPLE_BATCH_SIZE = 2048
EDGE_CHUNK_SIZE = 4096
TORCH_EPSILON = 1e-6

# Optimization: Thread pool settings
MAX_WORKERS = 4  # CPU threads - don't match robot count! (This was a major bottleneck)
REPLAN_QUEUE_SIZE = 5  # Max 5 pending replans in queue
MAX_REPLANS_PER_FRAME = 5  # Limit frame lag from replan results
COLLISION_LOOKAHEAD_FRAMES = 3  # Predict ahead 3 frames to enable proactive replanning

# Colors
WHITE = (255, 255, 255)
GRAY = (128, 128, 128)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)


def choose_prm_device():
    requested = os.environ.get("PRM_DEVICE", "").strip().lower()
    if requested == "mps" and torch.backends.mps.is_available():
        return torch.device("mps")
    if requested == "cuda" and torch.cuda.is_available():
        return torch.device("cuda")
    if requested == "cpu":
        return torch.device("cpu")
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


PRM_DEVICE = choose_prm_device()


# ============================================================================
# DATA STRUCTURES FOR NON-BLOCKING REPLAN SYSTEM
# ============================================================================

@dataclass
class ReplanJob:
    """Encapsulates a replan request"""
    robot_index: int
    position: Tuple[float, float]
    goal: Tuple[float, float]
    timestamp: float


@dataclass
class ReplanResult:
    """Holds the result of a completed replan"""
    robot_index: int
    path: Optional[List[Tuple[float, float]]]
    timestamp: float


class ReplanQueue:
    """Non-blocking replan queue with deduplication"""
    def __init__(self, max_size: int = 5):
        self.queue = Queue(maxsize=max_size)
        self.pending_robots = set()  # Track which robots have pending jobs
    
    def add_job(self, job: ReplanJob) -> bool:
        """Add a replan job, deduplicating if robot already has pending replan"""
        if job.robot_index in self.pending_robots:
            return False  # Already has a pending replan
        try:
            self.queue.put_nowait(job)
            self.pending_robots.add(job.robot_index)
            return True
        except:
            return False
    
    def get_job(self) -> Optional[ReplanJob]:
        """Retrieve next job without blocking"""
        try:
            job = self.queue.get_nowait()
            return job
        except:
            return None
    
    def mark_completed(self, robot_index: int):
        """Mark robot's replan as completed"""
        self.pending_robots.discard(robot_index)


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
    a = d[0] ** 2 + d[1] ** 2
    b = 2 * (f[0] * d[0] + f[1] * d[1])
    c = f[0] ** 2 + f[1] ** 2 - radius ** 2
    discriminant = b ** 2 - 4 * a * c
    if discriminant < 0:
        return False
    discriminant = math.sqrt(discriminant)
    t1 = (-b - discriminant) / (2 * a)
    t2 = (-b + discriminant) / (2 * a)
    if 0 <= t1 <= 1 or 0 <= t2 <= 1:
        return True
    return False


def check_robot_collisions_with_lookahead(robots, lookahead_frames: int = 1):
    """
    Predict future collisions by extrapolating robot positions.
    Returns set of robots that will collide in lookahead window.
    """
    colliding_robots = set()
    dt = 1.0 / 60.0  # Assuming 60 FPS
    
    for frame in range(lookahead_frames + 1):
        time_offset = frame * dt
        for i in range(len(robots)):
            # Extrapolate future position
            if robots[i].path and robots[i].current_waypoint_index < len(robots[i].path) - 1:
                current = robots[i].position
                target = robots[i].path[robots[i].current_waypoint_index + 1]
                dx = target[0] - current[0]
                dy = target[1] - current[1]
                future_pos_i = (
                    current[0] + dx * robots[i].speed * time_offset,
                    current[1] + dy * robots[i].speed * time_offset
                )
            else:
                future_pos_i = robots[i].position
            
            # Check against all other robots
            for j in range(i + 1, len(robots)):
                if robots[j].path and robots[j].current_waypoint_index < len(robots[j].path) - 1:
                    current = robots[j].position
                    target = robots[j].path[robots[j].current_waypoint_index + 1]
                    dx = target[0] - current[0]
                    dy = target[1] - current[1]
                    future_pos_j = (
                        current[0] + dx * robots[j].speed * time_offset,
                        current[1] + dy * robots[j].speed * time_offset
                    )
                else:
                    future_pos_j = robots[j].position
                
                # Check collision
                if dist(future_pos_i, future_pos_j) < robots[i].radius + robots[j].radius:
                    colliding_robots.add(i)
                    colliding_robots.add(j)
    
    return colliding_robots


def plan_path(obstacles, start, goal):
    """Worker thread function: builds PRM and finds path"""
    prm = PRM(obstacles, start, goal, N_SAMPLES, GAMMA_PRM, D)
    return prm.dijkstra()


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
    for index in range(len(polygon_vertices)):
        q1 = polygon_vertices[index]
        q2 = polygon_vertices[(index + 1) % len(polygon_vertices)]
        if segments_intersect(p1, p2, q1, q2):
            return True
    return False


def torch_points_in_polygon(points, polygon):
    if points.shape[0] == 0:
        return torch.zeros((0,), dtype=torch.bool, device=points.device)

    next_vertices = torch.roll(polygon, shifts=-1, dims=0)
    x = points[:, 0:1]
    y = points[:, 1:2]
    p1x = polygon[:, 0].unsqueeze(0)
    p1y = polygon[:, 1].unsqueeze(0)
    p2x = next_vertices[:, 0].unsqueeze(0)
    p2y = next_vertices[:, 1].unsqueeze(0)

    condition = (
        (y > torch.minimum(p1y, p2y))
        & (y <= torch.maximum(p1y, p2y))
        & (x <= torch.maximum(p1x, p2x))
    )
    denominator = torch.where(
        torch.abs(p2y - p1y) < TORCH_EPSILON,
        torch.ones_like(p2y - p1y),
        p2y - p1y,
    )
    intersections = (y - p1y) * (p2x - p1x) / denominator + p1x
    crossings = condition & ((torch.abs(p1x - p2x) < TORCH_EPSILON) | (x <= intersections))
    return (crossings.sum(dim=1) % 2) == 1


def torch_orientation(p, q, r):
    values = (q[..., 1] - p[..., 1]) * (r[..., 0] - q[..., 0]) - (q[..., 0] - p[..., 0]) * (r[..., 1] - q[..., 1])
    zeros = torch.zeros_like(values, dtype=torch.int8)
    ones = torch.ones_like(values, dtype=torch.int8)
    twos = torch.full_like(values, 2, dtype=torch.int8)
    return torch.where(torch.abs(values) <= TORCH_EPSILON, zeros, torch.where(values > 0, ones, twos))


def torch_on_segment(p, q, r):
    return (
        (q[..., 0] >= torch.minimum(p[..., 0], r[..., 0]) - TORCH_EPSILON)
        & (q[..., 0] <= torch.maximum(p[..., 0], r[..., 0]) + TORCH_EPSILON)
        & (q[..., 1] >= torch.minimum(p[..., 1], r[..., 1]) - TORCH_EPSILON)
        & (q[..., 1] <= torch.maximum(p[..., 1], r[..., 1]) + TORCH_EPSILON)
    )


def torch_segments_intersect_batch(segment_starts, segment_ends, edge_starts, edge_ends):
    if edge_starts.shape[0] == 0 or segment_starts.shape[0] == 0:
        return torch.zeros((segment_starts.shape[0],), dtype=torch.bool, device=segment_starts.device)

    p1 = segment_starts[:, None, :]
    q1 = segment_ends[:, None, :]
    p2 = edge_starts[None, :, :]
    q2 = edge_ends[None, :, :]

    o1 = torch_orientation(p1, q1, p2)
    o2 = torch_orientation(p1, q1, q2)
    o3 = torch_orientation(p2, q2, p1)
    o4 = torch_orientation(p2, q2, q1)

    general_case = (o1 != o2) & (o3 != o4)
    special_case_1 = (o1 == 0) & torch_on_segment(p1, p2, q1)
    special_case_2 = (o2 == 0) & torch_on_segment(p1, q2, q1)
    special_case_3 = (o3 == 0) & torch_on_segment(p2, p1, q2)
    special_case_4 = (o4 == 0) & torch_on_segment(p2, q1, q2)

    intersections = general_case | special_case_1 | special_case_2 | special_case_3 | special_case_4
    return intersections.any(dim=1)


class Robot:
    def __init__(self, position, radius, color, speed=50):
        self.position = position
        self.radius = radius
        self.color = color
        self.speed = speed
        self.path = None
        self.current_waypoint_index = 0
        self.reached_goal = False
        self.last_replan_time = 0
        # Visualization: show if robot is waiting for replan
        self.is_replanning = False

    def set_path(self, path):
        self.path = path
        self.current_waypoint_index = 0
        self.reached_goal = False

    def update(self, dt):
        if not self.path or self.reached_goal or self.current_waypoint_index >= len(self.path) - 1:
            self.reached_goal = True
            return

        current = self.position
        target = self.path[self.current_waypoint_index + 1]
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        distance_to_target = math.hypot(dx, dy)
        distance_to_move = self.speed * dt

        if distance_to_move >= distance_to_target:
            self.position = target
            self.current_waypoint_index += 1
        else:
            if distance_to_target > 0:
                ratio = distance_to_move / distance_to_target
                self.position = (
                    current[0] + dx * ratio,
                    current[1] + dy * ratio,
                )

    def get_target_waypoint(self):
        if self.path and self.current_waypoint_index < len(self.path) - 1:
            return self.path[self.current_waypoint_index + 1]
        return None


class Obstacle:
    def __init__(self, vertices, color):
        self.vertices = vertices
        self.color = color
        if vertices:
            center_x = sum(x for x, y in vertices) / len(vertices)
            center_y = sum(y for x, y in vertices) / len(vertices)
            self.bounding_center = (center_x, center_y)
            self.bounding_radius = max(dist(self.bounding_center, vertex) for vertex in vertices)
        else:
            self.bounding_center = (0, 0)
            self.bounding_radius = 0

    def get_inflated_vertices(self, inflation_factor=0.1):
        if not self.vertices:
            return []
        center_x = sum(x for x, y in self.vertices) / len(self.vertices)
        center_y = sum(y for x, y in self.vertices) / len(self.vertices)
        inflated = []
        for x, y in self.vertices:
            dx = x - center_x
            dy = y - center_y
            scale = 1 + inflation_factor
            inflated.append((center_x + dx * scale, center_y + dy * scale))
        return inflated


class Environment:
    def __init__(self, robots, obstacles, goal):
        self.robots = robots
        self.obstacles = obstacles
        self.goal = goal

    def render(self, screen):
        screen.fill(WHITE)
        for obstacle in self.obstacles:
            pygame.draw.polygon(screen, obstacle.color, obstacle.vertices)

        for robot in self.robots:
            if robot.path:
                for index in range(len(robot.path) - 1):
                    pygame.draw.line(screen, BLUE, robot.path[index], robot.path[index + 1], 2)
                for waypoint in robot.path:
                    pygame.draw.circle(screen, GRAY, (int(waypoint[0]), int(waypoint[1])), 3)

                target = robot.get_target_waypoint()
                if target:
                    pygame.draw.circle(screen, YELLOW, (int(target[0]), int(target[1])), 5)

        # Draw robots: color changes if waiting for replan
        for robot in self.robots:
            color = YELLOW if robot.is_replanning else robot.color
            pygame.draw.circle(screen, color, (int(robot.position[0]), int(robot.position[1])), robot.radius)

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
        self.device = PRM_DEVICE
        self.sample_seed = (
            (int(start[0]) * 73856093)
            ^ (int(start[1]) * 19349663)
            ^ (int(goal[0]) * 83492791)
            ^ (int(goal[1]) * 1234567)
            ^ n_samples
        ) & 0x7FFFFFFF
        self.sample_generator = torch.Generator(device="cpu")
        self.sample_generator.manual_seed(self.sample_seed)
        self.vertices = [start, goal]
        self.edges = {}
        self.weight_matrix = None
        self.vertex_tensor = None
        self._prepare_obstacle_tensors()
        self.build_graph()

    def _prepare_obstacle_tensors(self):
        self.inflated_polygons = []
        edge_starts = []
        edge_ends = []
        for obstacle in self.obstacles:
            inflated_vertices = obstacle.get_inflated_vertices(self.inflation_factor)
            if not inflated_vertices:
                continue
            polygon = torch.tensor(inflated_vertices, dtype=torch.float32, device=self.device)
            self.inflated_polygons.append(polygon)
            edge_starts.append(polygon)
            edge_ends.append(torch.roll(polygon, shifts=-1, dims=0))

        if edge_starts:
            self.obstacle_edge_starts = torch.cat(edge_starts, dim=0)
            self.obstacle_edge_ends = torch.cat(edge_ends, dim=0)
        else:
            self.obstacle_edge_starts = torch.empty((0, 2), dtype=torch.float32, device=self.device)
            self.obstacle_edge_ends = torch.empty((0, 2), dtype=torch.float32, device=self.device)

    def is_free(self, point):
        point_tensor = torch.tensor([point], dtype=torch.float32, device=self.device)
        return bool(self.is_free_batch(point_tensor)[0].item())

    def is_free_batch(self, points):
        if points.shape[0] == 0:
            return torch.zeros((0,), dtype=torch.bool, device=points.device)

        free_mask = torch.ones((points.shape[0],), dtype=torch.bool, device=points.device)
        for polygon in self.inflated_polygons:
            free_mask &= ~torch_points_in_polygon(points, polygon)
        return free_mask

    def generate_free_samples(self):
        collected = []
        collected_count = 0

        while collected_count < self.n_samples:
            sample_batch = torch.rand((SAMPLE_BATCH_SIZE, 2), generator=self.sample_generator, dtype=torch.float32)
            sample_batch[:, 0] *= WINDOW_WIDTH
            sample_batch[:, 1] *= WINDOW_HEIGHT
            sample_batch = sample_batch.to(self.device)

            free_samples = sample_batch[self.is_free_batch(sample_batch)]
            if free_samples.shape[0] == 0:
                continue

            remaining = self.n_samples - collected_count
            accepted = free_samples[:remaining]
            collected.append(accepted)
            collected_count += accepted.shape[0]

        return torch.cat(collected, dim=0)

    def filter_collision_free_edges(self, candidate_indices):
        if candidate_indices.shape[0] == 0:
            return candidate_indices

        valid_chunks = []
        for start_index in range(0, candidate_indices.shape[0], EDGE_CHUNK_SIZE):
            chunk = candidate_indices[start_index : start_index + EDGE_CHUNK_SIZE]
            segment_starts = self.vertex_tensor[chunk[:, 0]]
            segment_ends = self.vertex_tensor[chunk[:, 1]]
            intersects = torch_segments_intersect_batch(
                segment_starts,
                segment_ends,
                self.obstacle_edge_starts,
                self.obstacle_edge_ends,
            )
            valid_chunks.append(chunk[~intersects])

        return torch.cat(valid_chunks, dim=0) if valid_chunks else candidate_indices[:0]

    def build_graph(self):
        sample_tensor = self.generate_free_samples()
        start_goal_tensor = torch.tensor([self.start, self.goal], dtype=torch.float32, device=self.device)
        self.vertex_tensor = torch.cat((start_goal_tensor, sample_tensor), dim=0)
        self.vertices = [self.start, self.goal] + [tuple(point) for point in sample_tensor.cpu().tolist()]

        vertex_count = self.vertex_tensor.shape[0]
        connection_radius = self.gamma * (math.log(vertex_count) / vertex_count) ** (1 / self.d)
        distance_matrix = torch.cdist(self.vertex_tensor, self.vertex_tensor)
        upper_triangle = torch.triu(
            torch.ones((vertex_count, vertex_count), dtype=torch.bool, device=self.device),
            diagonal=1,
        )
        candidate_mask = (distance_matrix <= connection_radius) & upper_triangle
        candidate_indices = torch.nonzero(candidate_mask, as_tuple=False)
        valid_indices = self.filter_collision_free_edges(candidate_indices)

        self.weight_matrix = torch.full(
            (vertex_count, vertex_count),
            float("inf"),
            dtype=torch.float32,
            device=self.device,
        )
        diagonal_indices = torch.arange(vertex_count, device=self.device)
        self.weight_matrix[diagonal_indices, diagonal_indices] = 0.0

        if valid_indices.shape[0] == 0:
            return

        weights = distance_matrix[valid_indices[:, 0], valid_indices[:, 1]]
        self.weight_matrix[valid_indices[:, 0], valid_indices[:, 1]] = weights
        self.weight_matrix[valid_indices[:, 1], valid_indices[:, 0]] = weights

    def parallel_shortest_path(self):
        if self.weight_matrix is None:
            return None

        vertex_count = self.weight_matrix.shape[0]
        distances = torch.full((vertex_count,), float("inf"), dtype=torch.float32, device=self.device)
        predecessors = torch.full((vertex_count,), -1, dtype=torch.long, device=self.device)
        distances[0] = 0.0

        relax_weights = self.weight_matrix.clone()
        diagonal_indices = torch.arange(vertex_count, device=self.device)
        relax_weights[diagonal_indices, diagonal_indices] = float("inf")

        for _ in range(vertex_count - 1):
            candidate_distances = distances.unsqueeze(1) + relax_weights
            best_distances, best_predecessors = torch.min(candidate_distances, dim=0)
            improved = best_distances < distances
            if not bool(improved.any().item()):
                break
            distances = torch.where(improved, best_distances, distances)
            predecessors = torch.where(improved, best_predecessors, predecessors)

        if bool(torch.isinf(distances[1]).item()):
            return None

        predecessor_list = predecessors.cpu().tolist()
        path_indices = [1]
        current_index = 1
        seen = {1}

        while current_index != 0:
            current_index = predecessor_list[current_index]
            if current_index < 0 or current_index in seen:
                return None
            path_indices.append(current_index)
            seen.add(current_index)

        path_indices.reverse()
        return [self.vertices[index] for index in path_indices]

    def dijkstra(self):
        return self.parallel_shortest_path()


def generate_obstacles():
    obstacles = []
    for _ in range(OBSTACLE_COUNT):
        while True:
            num_sides = random.randint(3, 6)
            center_x = random.randint(50, WINDOW_WIDTH - 50)
            center_y = random.randint(50, WINDOW_HEIGHT - 50)
            radius = random.randint(20, 50)
            vertices = []
            for index in range(num_sides):
                angle = 2 * math.pi * index / num_sides + random.uniform(0, 2 * math.pi / num_sides)
                x = center_x + radius * math.cos(angle)
                y = center_y + radius * math.sin(angle)
                vertices.append((x, y))
            obstacle = Obstacle(vertices, GRAY)
            overlap = False
            for existing in obstacles:
                if dist(obstacle.bounding_center, existing.bounding_center) <= obstacle.bounding_radius + existing.bounding_radius:
                    overlap = True
                    break
            if not overlap:
                obstacles.append(obstacle)
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
            overlap = False
            for obstacle in obstacles:
                if dist(position, obstacle.bounding_center) <= ROBOT_RADIUS + obstacle.bounding_radius:
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
        overlap = False
        for obstacle in obstacles:
            if dist(position, obstacle.bounding_center) <= GOAL_RADIUS + obstacle.bounding_radius:
                overlap = True
                break
        if not overlap:
            for robot in robots:
                if dist(position, robot.position) <= GOAL_RADIUS + robot.radius:
                    overlap = True
                    break
        if not overlap:
            return goal


# ============================================================================
# MAIN LOOP WITH OPTIMIZATIONS
# ============================================================================

def main():
    random.seed(SEED)
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Motion Planning Environment (Optimized)")

    print(f"PRM compute device: {PRM_DEVICE.type}")
    print(f"Thread pool workers: {MAX_WORKERS}")
    print(f"Collision lookahead frames: {COLLISION_LOOKAHEAD_FRAMES}")

    obstacles = generate_obstacles()
    robots = generate_robots(obstacles)
    goal = generate_goal(obstacles, robots)
    env = Environment(robots, obstacles, goal)

    # OPTIMIZATION: Create thread pool once at startup, not per frame
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=MAX_WORKERS)
    replan_queue = ReplanQueue(max_size=REPLAN_QUEUE_SIZE)
    
    # Initial path planning for all robots
    for robot in env.robots:
        prm = PRM(env.obstacles, robot.position, env.goal[0], N_SAMPLES, GAMMA_PRM, D)
        path = prm.dijkstra()
        if path:
            robot.set_path(path)
            # print(f"Path found for robot at {robot.position}: length {len(path)}")
        else:
            # print(f"No path found for robot at {robot.position}")
            continue

    clock = pygame.time.Clock()
    fps = 60
    running = True
    frame_count = 0

    while running:
        dt = clock.tick(fps) / 1000.0
        frame_count += 1

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update robot positions
        for robot in env.robots:
            robot.update(dt)

        # OPTIMIZATION 1: Predictive collision detection with lookahead
        # Only check every few frames to reduce computation
        if frame_count % 3 == 0:
            colliding_robots = check_robot_collisions_with_lookahead(
                env.robots, 
                lookahead_frames=COLLISION_LOOKAHEAD_FRAMES
            )
        else:
            colliding_robots = set()

        # OPTIMIZATION 2: Queue replans asynchronously instead of blocking
        if colliding_robots:
            for idx in colliding_robots:
                robot = env.robots[idx]
                job = ReplanJob(
                    robot_index=idx,
                    position=robot.position,
                    goal=env.goal[0],
                    timestamp=frame_count
                )
                if replan_queue.add_job(job):
                    robot.is_replanning = True
                    # print(f"Queued replan for robot {idx}")

        # OPTIMIZATION 3: Process replan results non-blocking, max 2 per frame
        replans_processed = 0
        while replans_processed < MAX_REPLANS_PER_FRAME:
            job = replan_queue.get_job()
            if job is None:
                break
            
            # Execute the replan on thread pool
            try:
                future = executor.submit(plan_path, env.obstacles, job.position, job.goal)
                # Wait with timeout to avoid blocking
                path = future.result(timeout=0.05)
                
                if path:
                    env.robots[job.robot_index].set_path(path)
                    # print(f"Replanned path for robot {job.robot_index}: length {len(path)}")
                else:
                    # print(f"No path found for robot {job.robot_index} during replanning")
                    continue
                
                env.robots[job.robot_index].is_replanning = False
                replan_queue.mark_completed(job.robot_index)
                replans_processed += 1
            except concurrent.futures.TimeoutError:
                # Put job back in queue if it times out
                print(f"Replan timeout for robot {job.robot_index}, requeueing...")
                replan_queue.queue.put(job)
                break
            except Exception as e:
                print(f"Error replanning robot {job.robot_index}: {e}")
                env.robots[job.robot_index].is_replanning = False
                replan_queue.mark_completed(job.robot_index)

        # Render
        env.render(screen)
        pygame.display.flip()

    executor.shutdown(wait=True)
    pygame.quit()


if __name__ == "__main__":
    main()