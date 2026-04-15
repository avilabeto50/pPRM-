# pPRM- (Parallel Probabilistic Roadmap Method)

A 2D motion planning environment implemented in Python using Pygame, designed for testing and visualizing pathfinding algorithms like PRM* (Probabilistic Roadmap Method).

## Features

- **Environment Setup**: 800x800 pixel window with randomly generated polygon obstacles
- **Robots**: 5 red circular robots (radius 15px) spawned on the left half of the screen
- **Goal**: A single green circular goal (radius 20px) on the right half
- **Obstacles**: At least 5 gray polygon obstacles of varying sizes and shapes
- **Collision Detection**: Built-in functions for circle-polygon collision checking
- **Reproducibility**: Seeded random generation for consistent results

## Requirements

- Python 3.x
- Pygame: `pip install pygame`

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/avilabeto50/pPRM-.git
   cd pPRM-
   ```

2. Install dependencies:
   ```bash
   pip install pygame
   ```

## Usage

Run the environment visualization:
```bash
python motion_planning_env.py
```

This will open a Pygame window displaying the randomly generated environment. Close the window to exit.

## Configuration

All key parameters are defined as constants at the top of `motion_planning_env.py`:

- `SEED`: Random seed for reproducible generation (default: 42)
- `WINDOW_WIDTH`, `WINDOW_HEIGHT`: Window dimensions (default: 800x800)
- `ROBOT_COUNT`: Number of robots (default: 5)
- `ROBOT_RADIUS`: Robot radius in pixels (default: 15)
- `OBSTACLE_COUNT`: Number of obstacles (default: 5)
- `GOAL_RADIUS`: Goal radius in pixels (default: 20)

Modify these constants to customize the environment.

## Code Structure

- `Robot`: Class representing a robot with position, radius, and color
- `Obstacle`: Class representing a polygon obstacle with vertices and color
- `Environment`: Class managing robots, obstacles, and goal, with rendering functionality
- `check_collision()`: Function to detect collisions between circles and polygons
- Generation functions: `generate_obstacles()`, `generate_robots()`, `generate_goal()` ensure no initial overlaps

## Future Extensions

This environment is designed as a foundation for implementing motion planning algorithms such as:
- Probabilistic Roadmap Method (PRM)
- PRM* (optimal PRM)
- Parallel variants for multi-robot planning

## License

[Add license information here]
