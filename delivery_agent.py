import time
import logging
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum
from environment import GridEnvironment, Position
from search_algorithms import (SearchAlgorithm, BFS, UniformCostSearch, AStar, 
                             HillClimbingSearch, SimulatedAnnealingSearch, SearchResult)

class PlanningStrategy(Enum):
    """Available planning strategies."""
    BFS = "bfs"
    UNIFORM_COST = "uniform_cost"
    ASTAR_MANHATTAN = "astar_manhattan"
    ASTAR_EUCLIDEAN = "astar_euclidean"
    ASTAR_DIAGONAL = "astar_diagonal"
    HILL_CLIMBING = "hill_climbing"
    SIMULATED_ANNEALING = "simulated_annealing"

@dataclass
class DeliveryTask:
    """Represents a delivery task."""
    package_id: str
    pickup_location: Position
    delivery_location: Position
    priority: int = 1  # Higher number = higher priority
    deadline: Optional[int] = None  # Time deadline for delivery

@dataclass
class AgentState:
    """Current state of the delivery agent."""
    position: Position
    time: int
    fuel: float
    carrying_packages: List[str]
    completed_deliveries: List[str]

class DeliveryAgent:
    """Autonomous delivery agent with multiple planning strategies."""
    
    def __init__(self, environment: GridEnvironment, initial_position: Position,
                 initial_fuel: float = 100.0, fuel_consumption_rate: float = 1.0):
        self.env = environment
        self.initial_position = initial_position
        self.initial_fuel = initial_fuel
        self.fuel_consumption_rate = fuel_consumption_rate

        self.state = AgentState(
            position=initial_position,
            time=0,
            fuel=initial_fuel,
            carrying_packages=[],
            completed_deliveries=[]
        )
        self.tasks: List[DeliveryTask] = []
        self.current_strategy = PlanningStrategy.ASTAR_MANHATTAN
        self.planning_algorithm: Optional[SearchAlgorithm] = None
        self.current_path: List[Position] = []
        self.path_index = 0
        # Logging setup
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        # Statistics
        self.stats = {
            'total_distance_traveled': 0.0,
            'total_fuel_consumed': 0.0,
            'total_planning_time': 0.0,
            'replanning_events': 0,
            'deliveries_completed': 0,
            'tasks_failed': 0
        }
    def set_planning_strategy(self, strategy: PlanningStrategy):
        """Set the planning strategy for the agent."""
        self.current_strategy = strategy
        self.planning_algorithm = self._create_algorithm(strategy)
        self.logger.info(f"Switched to planning strategy: {strategy.value}")
    
    def _create_algorithm(self, strategy: PlanningStrategy) -> SearchAlgorithm:
        """Create search algorithm based on strategy."""
        if strategy == PlanningStrategy.BFS:
            return BFS(self.env)
        elif strategy == PlanningStrategy.UNIFORM_COST:
            return UniformCostSearch(self.env)
        elif strategy == PlanningStrategy.ASTAR_MANHATTAN:
            return AStar(self.env, AStar(self.env).manhattan_distance)
        elif strategy == PlanningStrategy.ASTAR_EUCLIDEAN:
            return AStar(self.env, AStar(self.env).euclidean_distance)
        elif strategy == PlanningStrategy.ASTAR_DIAGONAL:
            return AStar(self.env, AStar(self.env).diagonal_distance)
        elif strategy == PlanningStrategy.HILL_CLIMBING:
            return HillClimbingSearch(self.env)
        elif strategy == PlanningStrategy.SIMULATED_ANNEALING:
            return SimulatedAnnealingSearch(self.env)
        else:
            raise ValueError(f"Unknown planning strategy: {strategy}")
    
    def add_delivery_task(self, task: DeliveryTask):
        """Add a new delivery task."""
        self.tasks.append(task)
        self.tasks.sort(key=lambda t: t.priority, reverse=True)
        self.logger.info(f"Added delivery task: {task.package_id} from {task.pickup_location} to {task.delivery_location}")
    
    def plan_next_action(self) -> bool:
        """Plan the next action for the agent. Returns True if planning successful."""
        if not self.planning_algorithm:
            self.planning_algorithm = self._create_algorithm(self.current_strategy)
        start_time = time.time()
        # Determine next goal
        next_goal = self._determine_next_goal()
        if not next_goal:
            self.logger.info("No more tasks to complete")
            return False
        # Plan path to goal
        result = self.planning_algorithm.search(
            self.state.position, 
            next_goal, 
            self.state.time
        )
        planning_time = time.time() - start_time
        self.stats['total_planning_time'] += planning_time
        if result.success:
            self.current_path = result.path[1:]  # Exclude current position
            self.path_index = 0
            self.logger.info(f"Planned path to {next_goal}: {len(self.current_path)} steps, "
                           f"cost: {result.cost:.2f}, planning time: {planning_time:.4f}s")
            return True
        else:
            self.logger.warning(f"Failed to plan path to {next_goal}")
            return False
    
    def _determine_next_goal(self) -> Optional[Position]:
        """Determine the next goal position based on current state and tasks."""
        # If carrying packages, deliver them first
        if self.state.carrying_packages:
            for task in self.tasks:
                if (task.package_id in self.state.carrying_packages and 
                    task.delivery_location != self.state.position):
                    return task.delivery_location
        # Otherwise, pick up the highest priority available package
        for task in self.tasks:
            if (task.package_id not in self.state.completed_deliveries and 
                task.package_id not in self.state.carrying_packages and
                task.pickup_location != self.state.position):
                return task.pickup_location
        return None
    
    def execute_next_action(self) -> bool:
        """Execute the next action in the current path. Returns True if action executed."""
        if not self.current_path or self.path_index >= len(self.current_path):
            return False
        next_position = self.current_path[self.path_index]
        # Check if path is still valid (dynamic obstacles may have moved)
        if not self._is_path_still_valid():
            self.logger.info("Path no longer valid, replanning...")
            self.stats['replanning_events'] += 1
            return self.plan_next_action()
        # Move to next position
        self._move_to_position(next_position)
        self.path_index += 1
        # Check for task completion
        self._check_task_completion()
        return True
    
    def _is_path_still_valid(self) -> bool:
        """Check if the remaining path is still valid."""
        if not self.current_path or self.path_index >= len(self.current_path):
            return False
        for i in range(self.path_index, len(self.current_path)):
            pos = self.current_path[i]
            if not self.env.is_passable(pos.x, pos.y, self.state.time + (i - self.path_index)):
                return False
        return True
       
    def _move_to_position(self, position: Position):
        """Move agent to the specified position."""
        if position == self.state.position:
            return
        # Calculate movement cost
        cost = self.env.get_movement_cost(position.x, position.y)
        # Update state
        self.state.position = position
        self.state.time += 1
        self.state.fuel -= cost * self.fuel_consumption_rate
        # Update statistics
        self.stats['total_distance_traveled'] += cost
        self.stats['total_fuel_consumed'] += cost * self.fuel_consumption_rate
        self.logger.debug(f"Moved to {position}, fuel: {self.state.fuel:.2f}")
    
    def _check_task_completion(self):
        """Check if any tasks can be completed at current position."""
        for task in self.tasks:
            if task.package_id in self.state.completed_deliveries:
                continue
            # Pick up package
            if (task.pickup_location == self.state.position and 
                task.package_id not in self.state.carrying_packages):
                self.state.carrying_packages.append(task.package_id)
                self.logger.info(f"Picked up package {task.package_id}")
            # Deliver package
            elif (task.delivery_location == self.state.position and 
                  task.package_id in self.state.carrying_packages):
                self.state.carrying_packages.remove(task.package_id)
                self.state.completed_deliveries.append(task.package_id)
                self.stats['deliveries_completed'] += 1
                self.logger.info(f"Delivered package {task.package_id}")
    
    def run_simulation(self, max_steps: int = 1000) -> Dict[str, Any]:
        """Run the delivery simulation for a specified number of steps."""
        self.logger.info("Starting delivery simulation...")
        step = 0
        while step < max_steps and self.state.fuel > 0:
            # Check if we need to plan
            if not self.current_path or self.path_index >= len(self.current_path):
                if not self.plan_next_action():
                    break  # No more tasks
            # Execute next action
            if not self.execute_next_action():
                break
            
            step += 1
            # Advance environment time
            self.env.advance_time()
        # Final statistics
        self.stats['simulation_steps'] = step
        self.stats['final_fuel'] = self.state.fuel
        self.stats['final_position'] = self.state.position
        self.stats['tasks_remaining'] = len(self.tasks) - len(self.state.completed_deliveries)
        self.logger.info(f"Simulation completed after {step} steps")
        self.logger.info(f"Deliveries completed: {self.stats['deliveries_completed']}")
        self.logger.info(f"Fuel remaining: {self.state.fuel:.2f}")
        return self.stats.copy()
    
    def reset(self):
        """Reset the agent to initial state."""
        self.state = AgentState(
            position=self.initial_position,
            time=0,
            fuel=self.initial_fuel,
            carrying_packages=[],
            completed_deliveries=[]
        )
        self.current_path = []
        self.path_index = 0
        self.stats = {
            'total_distance_traveled': 0.0,
            'total_fuel_consumed': 0.0,
            'total_planning_time': 0.0,
            'replanning_events': 0,
            'deliveries_completed': 0,
            'tasks_failed': 0
        }
        self.env.current_time = 0
    
    def get_status(self) -> Dict[str, Any]:
        """Get current agent status."""
        return {
            'position': self.state.position,
            'time': self.state.time,
            'fuel': self.state.fuel,
            'carrying_packages': self.state.carrying_packages,
            'completed_deliveries': self.state.completed_deliveries,
            'current_path_length': len(self.current_path) - self.path_index if self.current_path else 0,
            'strategy': self.current_strategy.value
        }
def create_test_scenario(env: GridEnvironment) -> List[DeliveryTask]:
    """Create a test scenario with multiple delivery tasks."""
    tasks = [
        DeliveryTask("package_1", Position(1, 1), Position(8, 8), priority=3),
        DeliveryTask("package_2", Position(2, 2), Position(7, 7), priority=2),
        DeliveryTask("package_3", Position(3, 3), Position(6, 6), priority=1),
    ]
    return tasks

if __name__ == "__main__":
    # Test the delivery agent
    from environment import create_test_environment_small
    env = create_test_environment_small()
    agent = DeliveryAgent(env, Position(0, 0), initial_fuel=50.0)
    # Add some test tasks
    tasks = create_test_scenario(env)
    for task in tasks:
        agent.add_delivery_task(task)
    # Test different strategies
    strategies = [PlanningStrategy.BFS, PlanningStrategy.ASTAR_MANHATTAN, 
                 PlanningStrategy.HILL_CLIMBING]
    
    for strategy in strategies:
        print(f"\nTesting {strategy.value}:")
        agent.reset()
        agent.set_planning_strategy(strategy)
        stats = agent.run_simulation(max_steps=100)
        print(f"Completed {stats['deliveries_completed']} deliveries in {stats['simulation_steps']} steps")
