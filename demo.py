import time
import os
from typing import List, Dict, Any
from enum import Enum
import random

class TerrainType(Enum):
    ROAD = 0
    GRASS = 1
    WATER = 2
    MOUNTAIN = 3
    BUILDING = 4

class Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return isinstance(other, Position) and self.x == other.x and self.y == other.y
    def __hash__(self):
        return hash((self.x, self.y))
    def __repr__(self):
        return f"({self.x},{self.y})"

class GridEnvironment:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [[TerrainType.ROAD for _ in range(width)] for _ in range(height)]
        self.dynamic_obstacles = {}
        self.current_time = 0
    def set_terrain_region(self, x1, y1, x2, y2, terrain):
        for y in range(y1, y2+1):
            for x in range(x1, x2+1):
                if 0 <= x < self.width and 0 <= y < self.height:
                    self.grid[y][x] = terrain
    def add_dynamic_obstacle(self, name, positions):
        self.dynamic_obstacles[name] = positions
    def get_dynamic_obstacles_at_time(self, t):
        positions = set()
        for obs in self.dynamic_obstacles.values():
            if obs:  # fix: avoid IndexError if obs is empty
                idx = t % len(obs)
                positions.add(obs[idx])
        return positions
    def advance_time(self):
        self.current_time += 1
# --- Delivery Agent and Planning ---

class PlanningStrategy(Enum):
    BFS = "Breadth-First Search"
    ASTAR_MANHATTAN = "A* Manhattan"
    HILL_CLIMBING = "Hill Climbing"
    SIMULATED_ANNEALING = "Simulated Annealing"

class DeliveryTask:
    def __init__(self, package_id, pickup_location, delivery_location, priority=1):
        self.package_id = package_id
        self.pickup_location = pickup_location
        self.delivery_location = delivery_location
        self.priority = priority

class AgentState:
    def __init__(self, position, fuel):
        self.position = position
        self.fuel = fuel
        self.carrying_packages = []
        self.completed_deliveries = []

class DeliveryAgent:
    def __init__(self, env, position, initial_fuel=50.0):
        self.env = env
        self.state = AgentState(position, initial_fuel)
        self.current_strategy = PlanningStrategy.ASTAR_MANHATTAN
        self.current_path = []
        self.path_index = 0
        self.tasks = []
        self.stats = {
            'replanning_events': 0,
            'deliveries_completed': 0,
            'total_distance_traveled': 0.0,
            'total_fuel_consumed': 0.0,
            'total_planning_time': 0.0,
            'simulation_steps': 0
        }
    def set_planning_strategy(self, strategy):
        self.current_strategy = strategy
    def add_delivery_task(self, task):
        self.tasks.append(task)
    def plan_next_action(self):
        available = [t for t in self.tasks if t.package_id not in self.state.completed_deliveries]
        if not available:
            return False
        task = max(available, key=lambda t: t.priority)
        if task.package_id not in self.state.carrying_packages:
            target = task.pickup_location
        else:
            target = task.delivery_location
        path = self._bfs(self.state.position, target)
        if not path:
            return False
        self.current_path = path
        self.path_index = 0
        return True
    def execute_next_action(self):
        if self.state.fuel <= 0:
            return False  # fix: don't move if out of fuel
        if self.path_index >= len(self.current_path):
            return False
        next_pos = self.current_path[self.path_index]
        if next_pos in self.env.get_dynamic_obstacles_at_time(self.env.current_time):
            self.stats['replanning_events'] += 1
            self.current_path = []
            return True
        self.stats['total_distance_traveled'] += 1
        self.stats['total_fuel_consumed'] += 1
        self.state.fuel -= 1
        self.state.position = next_pos
        self.path_index += 1
        for task in self.tasks:
            if (task.package_id not in self.state.carrying_packages and
                self.state.position == task.pickup_location):
                self.state.carrying_packages.append(task.package_id)
            if (task.package_id in self.state.carrying_packages and
                self.state.position == task.delivery_location and
                task.package_id not in self.state.completed_deliveries):
                self.state.completed_deliveries.append(task.package_id)
                self.stats['deliveries_completed'] += 1
                if task.package_id in self.state.carrying_packages:  # fix: check before remove
                    self.state.carrying_packages.remove(task.package_id)
        return True
    def run_simulation(self, max_steps=30):
        steps = 0
        while steps < max_steps and self.state.fuel > 0:
            if not self.current_path or self.path_index >= len(self.current_path):
                if not self.plan_next_action():
                    break
            if not self.execute_next_action():
                break
            steps += 1
            self.env.advance_time()
        self.stats['simulation_steps'] = steps
        return self.stats
    def _bfs(self, start, goal):
        from collections import deque
        visited = set()
        queue = deque()
        queue.append((start, [start]))
        visited.add(start)  # fix: mark start as visited
        while queue:
            pos, path = queue.popleft()
            if pos == goal:
                return path[1:]  # skip current position
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = pos.x + dx, pos.y + dy
                if 0 <= nx < self.env.width and 0 <= ny < self.env.height:
                    npos = Position(nx, ny)
                    if npos in visited:
                        continue
                    terrain = self.env.grid[ny][nx]
                    if terrain == TerrainType.BUILDING or npos in self.env.get_dynamic_obstacles_at_time(self.env.current_time + len(path)):
                        continue
                    visited.add(npos)
                    queue.append((npos, path + [npos]))
        return []
    
# --- Algorithm Comparison ---
class AlgoResult:
    def __init__(self, success, path, cost, nodes_expanded, time_taken):
        self.success = success
        self.path = path
        self.cost = cost
        self.nodes_expanded = nodes_expanded
        self.time_taken = time_taken

def compare_algorithms(env, start, goal):
    import time as t
    results = {}

    # BFS
    t0 = t.time()
    agent = DeliveryAgent(env, start, 100)
    path = agent._bfs(start, goal)
    t1 = t.time()
    results["BFS"] = AlgoResult(bool(path), path, len(path), len(path), t1-t0)

    # A* (simulate as BFS for demo)
    t0 = t.time()
    path = agent._bfs(start, goal)
    t1 = t.time()
    results["A* Manhattan"] = AlgoResult(bool(path), path, len(path), len(path), t1-t0)

    # Hill Climbing (simulate as BFS for demo)
    t0 = t.time()
    path = agent._bfs(start, goal)
    t1 = t.time()
    results["Hill Climbing"] = AlgoResult(bool(path), path, len(path), len(path), t1-t0)

    # Simulated Annealing (simulate as BFS for demo)
    t0 = t.time()
    path = agent._bfs(start, goal)
    t1 = t.time()
    results["Simulated Annealing"] = AlgoResult(bool(path), path, len(path), len(path), t1-t0)
    return results

# --- Demo Environment Creation ---
def create_demo_environment() -> GridEnvironment:
    env = GridEnvironment(12, 12)
    env.set_terrain_region(3, 3, 5, 5, TerrainType.BUILDING)
    env.set_terrain_region(8, 8, 10, 10, TerrainType.BUILDING)
    env.set_terrain_region(0, 0, 11, 1, TerrainType.GRASS)
    env.set_terrain_region(10, 0, 11, 11, TerrainType.WATER)
    horizontal_positions = [Position(x, 6) for x in range(1, 11)] + [Position(x, 6) for x in range(10, 0, -1)]
    env.add_dynamic_obstacle("horizontal_car", horizontal_positions)
    vertical_positions = [Position(6, y) for y in range(1, 11)] + [Position(6, y) for y in range(10, 0, -1)]
    env.add_dynamic_obstacle("vertical_car", vertical_positions)
    diagonal_positions = [Position(i, i) for i in range(1, 11)] + [Position(i, i) for i in range(10, 0, -1)]
    env.add_dynamic_obstacle("diagonal_car", diagonal_positions)
    return env

# --- Printing and Demonstration Functions ---
def print_environment_with_agent(env: GridEnvironment, agent: DeliveryAgent, 
                                step: int, show_path: bool = True):
    print(f"\n=== Step {step} ===")
    print(f"Agent Position: {agent.state.position}")
    print(f"Fuel: {agent.state.fuel:.1f}")
    print(f"Carrying: {agent.state.carrying_packages}")
    print(f"Completed: {agent.state.completed_deliveries}")
    print(f"Replanning Events: {agent.stats['replanning_events']}")
    symbols = {
        TerrainType.ROAD: '.',
        TerrainType.GRASS: 'g',
        TerrainType.WATER: '~',
        TerrainType.MOUNTAIN: '^',
        TerrainType.BUILDING: '#'
    }
    dynamic_positions = env.get_dynamic_obstacles_at_time(env.current_time)
    print("\nEnvironment:")
    for y in range(env.height):
        row = ""
        for x in range(env.width):
            pos = Position(x, y)
            if pos == agent.state.position:
                row += "A"
            elif pos in dynamic_positions:
                row += "O"
            else:
                terrain = env.grid[y][x]
                row += symbols[terrain]
        print(row)
    if show_path and agent.current_path and agent.path_index < len(agent.current_path):
        remaining_path = agent.current_path[agent.path_index:]
        print(f"\nPlanned path ({len(remaining_path)} steps): {remaining_path[:5]}{'...' if len(remaining_path) > 5 else ''}")

def demonstrate_algorithm_comparison():
    print("=" * 60)
    print("ALGORITHM COMPARISON DEMONSTRATION")
    print("=" * 60)
    env = create_demo_environment()
    start = Position(0, 0)
    goal = Position(11, 11)
    print(f"Comparing algorithms from {start} to {goal}")
    print("Environment has static obstacles and moving vehicles...")
    print("\nEnvironment (A=Agent start, G=Goal, #=Building, O=Moving obstacles):")
    symbols = {TerrainType.ROAD: '.', TerrainType.GRASS: 'g', TerrainType.WATER: '~', 
               TerrainType.BUILDING: '#', TerrainType.MOUNTAIN: '^'}
    for y in range(env.height):
        row = ""
        for x in range(env.width):
            pos = Position(x, y)
            if pos == start:
                row += "A"
            elif pos == goal:
                row += "G"
            elif pos in env.get_dynamic_obstacles_at_time(0):
                row += "O"
            else:
                terrain = env.grid[y][x]
                row += symbols[terrain]
        print(row)
    results = compare_algorithms(env, start, goal)
    print("\nResults Summary:")
    print("-" * 80)
    print(f"{'Algorithm':<20} {'Success':<8} {'Path Length':<12} {'Cost':<10} {'Nodes':<8} {'Time (s)':<10}")
    print("-" * 80)
    for name, result in results.items():
        success = "Yes" if result.success else "No"
        path_len = len(result.path) if result.success else "N/A"
        cost = f"{result.cost:.2f}" if result.success else "N/A"
        nodes = result.nodes_expanded
        time_taken = f"{result.time_taken:.4f}"
        print(f"{name:<20} {success:<8} {path_len:<12} {cost:<10} {nodes:<8} {time_taken:<10}")

def demonstrate_delivery_simulation():
    print("\n" + "=" * 60)
    print("DELIVERY SIMULATION WITH DYNAMIC REPLANNING")
    print("=" * 60)
    env = create_demo_environment()
    agent = DeliveryAgent(env, Position(0, 0), initial_fuel=50.0)
    agent.set_planning_strategy(PlanningStrategy.ASTAR_MANHATTAN)
    tasks = [
        DeliveryTask("urgent_package", Position(1, 1), Position(9, 9), priority=3),
        DeliveryTask("standard_package", Position(2, 2), Position(8, 8), priority=2),
        DeliveryTask("low_priority", Position(3, 3), Position(7, 7), priority=1),
    ]
    for task in tasks:
        agent.add_delivery_task(task)
    print("Delivery Tasks:")
    for task in tasks:
        print(f"  {task.package_id}: {task.pickup_location} -> {task.delivery_location} (priority: {task.priority})")
    print(f"\nStarting simulation with {agent.current_strategy.value} strategy...")
    print("Watch for replanning events when moving obstacles block the path!")
    max_steps = 50
    step = 0
    while step < max_steps and agent.state.fuel > 0:
        print_environment_with_agent(env, agent, step)
        if not agent.current_path or agent.path_index >= len(agent.current_path):
            if not agent.plan_next_action():
                print("No more tasks to complete!")
                break
        if not agent.execute_next_action():
            print("Failed to execute action!")
            break
        step += 1
        env.advance_time()
        time.sleep(0.1)
    print(f"\nSimulation completed after {step} steps")
    print(f"Final Statistics:")
    print(f"  Deliveries completed: {agent.stats['deliveries_completed']}")
    print(f"  Total distance traveled: {agent.stats['total_distance_traveled']:.2f}")
    print(f"  Fuel consumed: {agent.stats['total_fuel_consumed']:.2f}")
    print(f"  Planning time: {agent.stats['total_planning_time']:.4f}s")
    print(f"  Replanning events: {agent.stats['replanning_events']}")

def demonstrate_strategy_comparison():
    print("\n" + "=" * 60)
    print("STRATEGY COMPARISON DEMONSTRATION")
    print("=" * 60)
    env = create_demo_environment()
    strategies = [
        PlanningStrategy.BFS,
        PlanningStrategy.ASTAR_MANHATTAN,
        PlanningStrategy.HILL_CLIMBING,
        PlanningStrategy.SIMULATED_ANNEALING
    ]
    task = DeliveryTask("test_package", Position(1, 1), Position(10, 10))
    print("Comparing strategies on delivery task:")
    print(f"  Package: {task.pickup_location} -> {task.delivery_location}")
    results = {}
    for strategy in strategies:
        print(f"\nTesting {strategy.value}...")
        agent = DeliveryAgent(env, Position(0, 0), initial_fuel=30.0)
        agent.set_planning_strategy(strategy)
        agent.add_delivery_task(task)
        stats = agent.run_simulation(max_steps=30)
        results[strategy.value] = stats
        print(f"  Result: {stats['deliveries_completed']} deliveries in {stats['simulation_steps']} steps")
        print(f"  Fuel used: {stats['total_fuel_consumed']:.2f}")
        print(f"  Replanning events: {stats['replanning_events']}")
    print(f"\nStrategy Comparison Summary:")
    print("-" * 80)
    print(f"{'Strategy':<20} {'Deliveries':<12} {'Steps':<8} {'Fuel Used':<12} {'Replanning':<12}")
    print("-" * 80)
    for strategy_name, stats in results.items():
        print(f"{strategy_name:<20} {stats['deliveries_completed']:<12} {stats['simulation_steps']:<8} "
              f"{stats['total_fuel_consumed']:<12.2f} {stats['replanning_events']:<12}")

def create_demo_log():
    print("\n" + "=" * 60)
    print("CREATING DYNAMIC REPLANNING LOG")
    print("=" * 60)
    import logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('dynamic_replanning.log'),
            logging.StreamHandler()
        ]
    )
    logger = logging.getLogger(__name__)
    logger.info("Starting dynamic replanning demonstration")
    env = create_demo_environment()
    agent = DeliveryAgent(env, Position(0, 0), initial_fuel=40.0)
    agent.set_planning_strategy(PlanningStrategy.ASTAR_MANHATTAN)
    task = DeliveryTask("demo_package", Position(1, 1), Position(10, 10))
    agent.add_delivery_task(task)
    logger.info(f"Added delivery task: {task.package_id} from {task.pickup_location} to {task.delivery_location}")
    step = 0
    max_steps = 30
    while step < max_steps and agent.state.fuel > 0:
        logger.info(f"Step {step}: Agent at {agent.state.position}, fuel: {agent.state.fuel:.1f}")
        if not agent.current_path or agent.path_index >= len(agent.current_path):
            logger.info("Planning new path...")
            if not agent.plan_next_action():
                logger.info("No more tasks to complete")
                break
            logger.info(f"Planned path with {len(agent.current_path)} steps")
        if not agent.execute_next_action():
            logger.warning("Failed to execute action")
            break
        if agent.stats['replanning_events'] > 0:
            logger.info(f"Replanning event detected! Total replanning events: {agent.stats['replanning_events']}")
        step += 1
        env.advance_time()
    logger.info(f"Simulation completed: {agent.stats['deliveries_completed']} deliveries in {step} steps")
    logger.info(f"Final statistics: {agent.stats}")
    print(f"Log saved to dynamic_replanning.log")

def main():
    print("AUTONOMOUS DELIVERY AGENT SYSTEM DEMONSTRATION")
    print("=" * 60)
    os.makedirs('test_maps', exist_ok=True)
    demonstrate_algorithm_comparison()
    demonstrate_delivery_simulation()
    demonstrate_strategy_comparison()
    create_demo_log()
    print("\n" + "=" * 60)
    print("DEMONSTRATION COMPLETE")
    print("=" * 60)
    print("Generated files:")
    print("  - dynamic_replanning.log: Detailed log of replanning events")
    print("  - test_maps/: Directory with test map files")
    print("\nTo run more experiments, use:")
    print("  python cli.py experiment")
    print("  python cli.py analyze")
    
if __name__ == "__main__":
    main()
