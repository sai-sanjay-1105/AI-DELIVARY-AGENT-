import numpy as np
from typing import List, Tuple, Optional, Dict, Set
from dataclasses import dataclass
from enum import Enum
import json
import time

class TerrainType(Enum):
    """Types of terrain with different movement costs."""
    ROAD = 1
    GRASS = 2
    WATER = 3
    MOUNTAIN = 4
    BUILDING = 5  

@dataclass
class Position:
    """Represents a position in the grid."""
    x: int
    y: int
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __eq__(self, other):
        if not isinstance(other, Position):
            return False
        return self.x == other.x and self.y == other.y
    
    def __lt__(self, other):
        """Required for heapq comparison."""
        if not isinstance(other, Position):
            return NotImplemented
        return (self.x, self.y) < (other.x, other.y)
    
    def __le__(self, other):
        if not isinstance(other, Position):
            return NotImplemented
        return (self.x, self.y) <= (other.x, other.y)
    
    def __gt__(self, other):
        if not isinstance(other, Position):
            return NotImplemented
        return (self.x, self.y) > (other.x, other.y)
    
    def __ge__(self, other):
        if not isinstance(other, Position):
            return NotImplemented
        return (self.x, self.y) >= (other.x, other.y)
    
    def __add__(self, other):
        if not isinstance(other, Position):
            return NotImplemented
        return Position(self.x + other.x, self.y + other.y)
    
    def __repr__(self):
        return f"Position(x={self.x}, y={self.y})"

@dataclass
class DynamicObstacle:
    """Represents a moving obstacle with a deterministic schedule."""
    id: str
    positions: List[Position]  # List of positions over time
    current_time: int = 0
    
    def get_position_at_time(self, time: int) -> Position:
        """Get obstacle position at given time (cycles through schedule)."""
        if not self.positions:
            return self.positions[0]
        return self.positions[time % len(self.positions)]
    
    def advance_time(self):
        """Advance obstacle to next position in schedule."""
        self.current_time += 1

class GridEnvironment:
    """2D grid environment with static obstacles, terrain costs, and dynamic obstacles."""
    def __init__(self, width: int, height: int, allow_diagonal: bool = False):
        self.width = width
        self.height = height
        self.allow_diagonal = allow_diagonal
        self.grid = np.full((height, width), TerrainType.ROAD, dtype=TerrainType)
        self.terrain_costs = {
            TerrainType.ROAD: 1,
            TerrainType.GRASS: 2,
            TerrainType.WATER: 3,
            TerrainType.MOUNTAIN: 4,
            TerrainType.BUILDING: float('inf')  # Impassable
        }
        self.dynamic_obstacles: Dict[str, DynamicObstacle] = {}
        self.current_time = 0
        # Movement directions
        if allow_diagonal:
            # 8-connected movement (including diagonals)
            self.directions = [
                Position(0, 1),   # Up
                Position(0, -1),  # Down
                Position(-1, 0),  # Left
                Position(1, 0),   # Right
                Position(-1, 1),  # Up-Left
                Position(1, 1),   # Up-Right
                Position(-1, -1), # Down-Left
                Position(1, -1)   # Down-Right
            ]
        else:
            # 4-connected movement (cardinal directions only)
            self.directions = [
                Position(0, 1),   # Up
                Position(0, -1),  # Down
                Position(-1, 0),  # Left
                Position(1, 0)    # Right
            ]
    def set_terrain(self, x: int, y: int, terrain: TerrainType):
        """Set terrain type at given position."""
        if self.is_valid_position(x, y):
            self.grid[y][x] = terrain
    
    def set_terrain_region(self, x1: int, y1: int, x2: int, y2: int, terrain: TerrainType):
        """Set terrain type for a rectangular region."""
        for y in range(max(0, y1), min(self.height, y2 + 1)):
            for x in range(max(0, x1), min(self.width, x2 + 1)):
                self.grid[y][x] = terrain
    
    def add_dynamic_obstacle(self, obstacle_id: str, positions: List[Position]):
        """Add a dynamic obstacle with its movement schedule."""
        self.dynamic_obstacles[obstacle_id] = DynamicObstacle(obstacle_id, positions)
    
    def is_valid_position(self, x: int, y: int) -> bool:
        """Check if position is within grid bounds."""
        return 0 <= x < self.width and 0 <= y < self.height
    
    def is_passable(self, x: int, y: int, time: int = None) -> bool:
        """Check if position is passable at given time."""
        if not self.is_valid_position(x, y):
            return False
        # Check static obstacles
        terrain = self.grid[y][x]
        if terrain == TerrainType.BUILDING:
            return False
        # Check dynamic obstacles
        if time is not None:
            for obstacle in self.dynamic_obstacles.values():
                if obstacle.get_position_at_time(time) == Position(x, y):
                    return False
        return True
    
    def get_movement_cost(self, x: int, y: int) -> float:
        """Get movement cost for terrain at given position."""
        if not self.is_valid_position(x, y):
            return float('inf')
        terrain = self.grid[y][x]
        # Handle the case where terrain might not be in the dictionary
        if terrain in self.terrain_costs:
            return self.terrain_costs[terrain]
        else:
            # Default cost for unknown terrain
            return 1.0
    def get_neighbors(self, pos: Position, time: int = None) -> List[Tuple[Position, float]]:
        """Get valid neighboring positions with their movement costs."""
        neighbors = []
        for direction in self.directions:
            new_pos = pos + direction
            if self.is_passable(new_pos.x, new_pos.y, time):
                cost = self.get_movement_cost(new_pos.x, new_pos.y)
                
                # Adjust cost for diagonal movement
                if self.allow_diagonal and abs(direction.x) + abs(direction.y) == 2:
                    # Diagonal movement costs sqrt(2) times the base cost
                    cost *= 1.414  # sqrt(2) ≈ 1.414
                neighbors.append((new_pos, cost))
        return neighbors

    def get_dynamic_obstacles_at_time(self, time: int) -> Set[Position]:
        """Get all dynamic obstacle positions at given time."""
        positions = set()
        for obstacle in self.dynamic_obstacles.values():
            positions.add(obstacle.get_position_at_time(time))
        return positions
    
    def advance_time(self):
        """Advance environment time by one step."""
        self.current_time += 1
        for obstacle in self.dynamic_obstacles.values():
            obstacle.advance_time()
    
    def save_to_file(self, filename: str):
        """Save environment to file."""
        data = {
            'width': self.width,
            'height': self.height,
            'grid': [[terrain.value for terrain in row] for row in self.grid],
            'dynamic_obstacles': {
                obs_id: {
                    'id': obs.id,
                    'positions': [(pos.x, pos.y) for pos in obs.positions]
                }
                for obs_id, obs in self.dynamic_obstacles.items()
            }
        }
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
    
    @classmethod
    def load_from_file(cls, filename: str) -> 'GridEnvironment':
        """Load environment from file."""
        with open(filename, 'r') as f:
            data = json.load(f)
        
        env = cls(data['width'], data['height'])
        # Load grid
        for y, row in enumerate(data['grid']):
            for x, terrain_value in enumerate(row):
                env.grid[y][x] = TerrainType(terrain_value)   
        # Load dynamic obstacles
        for obs_data in data['dynamic_obstacles'].values():
            positions = [Position(x, y) for x, y in obs_data['positions']]
            env.add_dynamic_obstacle(obs_data['id'], positions)
        return env
    
    def print_grid(self, agent_pos: Position = None, goal_pos: Position = None, time: int = None):
        """Print grid with optional agent and goal positions."""
        symbols = {
            TerrainType.ROAD: '.',
            TerrainType.GRASS: 'g',
            TerrainType.WATER: '~',
            TerrainType.MOUNTAIN: '^',
            TerrainType.BUILDING: '#'
        }
        dynamic_positions = self.get_dynamic_obstacles_at_time(time) if time is not None else set()
        print(f"Grid (Time: {time if time is not None else self.current_time}):")
        for y in range(self.height):
            row = ""
            for x in range(self.width):
                if agent_pos and agent_pos.x == x and agent_pos.y == y:
                    row += "A"
                elif goal_pos and goal_pos.x == x and goal_pos.y == y:
                    row += "G"
                elif Position(x, y) in dynamic_positions:
                    row += "O"
                else:
                    row += symbols[self.grid[y][x]]
            print(row)
        print()

def create_test_environment_small() -> GridEnvironment:
    """Create a small test environment."""
    env = GridEnvironment(10, 10)
    # Add some buildings
    env.set_terrain_region(2, 2, 2, 4, TerrainType.BUILDING)
    env.set_terrain_region(5, 5, 7, 7, TerrainType.BUILDING)
    # Add different terrain types
    env.set_terrain_region(0, 0, 9, 1, TerrainType.GRASS)
    env.set_terrain_region(8, 8, 9, 9, TerrainType.WATER)
    return env

def create_test_environment_medium() -> GridEnvironment:
    """Create a medium test environment."""
    env = GridEnvironment(20, 20)
    # Add buildings
    env.set_terrain_region(5, 5, 8, 8, TerrainType.BUILDING)
    env.set_terrain_region(12, 12, 15, 15, TerrainType.BUILDING)
    env.set_terrain_region(2, 15, 4, 18, TerrainType.BUILDING)
    # Add terrain variety
    env.set_terrain_region(0, 0, 19, 2, TerrainType.GRASS)
    env.set_terrain_region(17, 0, 19, 19, TerrainType.WATER)
    env.set_terrain_region(10, 0, 12, 5, TerrainType.MOUNTAIN)
    return env

def create_test_environment_large() -> GridEnvironment:
    """Create a large test environment."""
    env = GridEnvironment(50, 50)
    # Add multiple building clusters
    for i in range(0, 50, 10):
        for j in range(0, 50, 10):
            if (i + j) % 20 == 0:  # Every other cluster
                env.set_terrain_region(i, j, i+3, j+3, TerrainType.BUILDING)
    # Add terrain patterns
    for i in range(0, 50, 5):
        env.set_terrain_region(i, 0, i+2, 49, TerrainType.GRASS)
    # Add water features
    env.set_terrain_region(40, 40, 49, 49, TerrainType.WATER)
    env.set_terrain_region(0, 40, 10, 49, TerrainType.WATER)
    return env

def create_test_environment_dynamic() -> GridEnvironment:
    """Create environment with dynamic obstacles."""
    env = GridEnvironment(15, 15)
    # Add some static obstacles
    env.set_terrain_region(5, 5, 7, 7, TerrainType.BUILDING)
    env.set_terrain_region(10, 10, 12, 12, TerrainType.BUILDING)
    # Add moving obstacles
    # Moving obstacle 1: horizontal movement
    positions1 = [Position(x, 3) for x in range(2, 13)] + [Position(x, 3) for x in range(12, 1, -1)]
    env.add_dynamic_obstacle("moving_car_1", positions1)
    # Moving obstacle 2: vertical movement
    positions2 = [Position(8, y) for y in range(2, 13)] + [Position(8, y) for y in range(12, 1, -1)]
    env.add_dynamic_obstacle("moving_car_2", positions2)
    # Moving obstacle 3: diagonal movement
    positions3 = [Position(i, i) for i in range(1, 14)] + [Position(i, i) for i in range(13, 0, -1)]
    env.add_dynamic_obstacle("moving_car_3", positions3)
    return env

if __name__ == "__main__":
    # Test the environment
    env = create_test_environment_small()
    env.print_grid()
    # Test dynamic obstacles
    env_dynamic = create_test_environment_dynamic()
    print("Dynamic environment at different times:")
    for t in range(5):
        env_dynamic.print_grid(time=t)
        env_dynamic.advance_time()
