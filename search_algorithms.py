import heapq
import random
import time
import numpy as np
from typing import List, Tuple, Optional, Dict, Set, Callable
from dataclasses import dataclass
from environment import GridEnvironment, Position, TerrainType

@dataclass
class SearchNode:
    """Node in search tree."""
    position: Position
    parent: Optional['SearchNode']
    cost: float
    heuristic: float = 0.0
    time: int = 0
    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

    def get_path(self) -> List[Position]:
        """Get path from start to this node."""
        path = []
        current = self
        while current:
            path.append(current.position)
            current = current.parent
        return list(reversed(path))

class SearchResult:
    """Result of a search operation."""
    def __init__(self, success: bool, path: List[Position] = None, 
                 nodes_expanded: int = 0, time_taken: float = 0.0, 
                 cost: float = 0.0):
        self.success = success
        self.path = path or []
        self.nodes_expanded = nodes_expanded
        self.time_taken = time_taken
        self.cost = cost

class SearchAlgorithm:
    """Base class for search algorithms."""
    def __init__(self, environment: GridEnvironment):
        self.env = environment
    def search(self, start: Position, goal: Position, 
               start_time: int = 0) -> SearchResult:
        """Perform search from start to goal."""
        raise NotImplementedError

class BFS(SearchAlgorithm):
    """Breadth-First Search implementation."""
    def search(self, start: Position, goal: Position, 
               start_time: int = 0) -> SearchResult:
        start_time_actual = time.time()
        nodes_expanded = 0
        if start == goal:
            return SearchResult(True, [start], 0, time.time() - start_time_actual, 0.0)
        queue = [(start, start_time)]
        visited = {(start, start_time)}
        parent_map = {}
        while queue:
            current_pos, current_time = queue.pop(0)
            nodes_expanded += 1
            if current_pos == goal:
                # Reconstruct path
                path = [goal]
                while (current_pos, current_time) in parent_map:
                    current_pos, current_time = parent_map[(current_pos, current_time)]
                    path.append(current_pos)
                path.reverse()
                # Calculate total cost
                total_cost = sum(self.env.get_movement_cost(pos.x, pos.y) 
                               for pos in path[1:])
                
                return SearchResult(True, path, nodes_expanded, 
                                 time.time() - start_time_actual, total_cost)
            # Explore neighbors
            for neighbor_pos, cost in self.env.get_neighbors(current_pos, current_time):
                next_time = current_time + 1
                state = (neighbor_pos, next_time)
                if state not in visited:
                    visited.add(state)
                    parent_map[state] = (current_pos, current_time)
                    queue.append((neighbor_pos, next_time))
        return SearchResult(False, [], nodes_expanded, time.time() - start_time_actual)

class UniformCostSearch(SearchAlgorithm):
    """Uniform-Cost Search implementation."""
    def search(self, start: Position, goal: Position, 
               start_time: int = 0) -> SearchResult:
        start_time_actual = time.time()
        nodes_expanded = 0
        if start == goal:
            return SearchResult(True, [start], 0, time.time() - start_time_actual, 0.0)
        # Priority queue: (total_cost, position, time, parent)
        pq = [(0, start, start_time, None)]
        visited = set()
        cost_map = {(start, start_time): 0}
        parent_map = {}
        while pq:
            current_cost, current_pos, current_time, parent = heapq.heappop(pq)
            nodes_expanded += 1
            if current_pos == goal:
                # Reconstruct path
                path = [goal]
                current_state = (current_pos, current_time)
                while current_state in parent_map:
                    current_state = parent_map[current_state]
                    path.append(current_state[0])
                path.reverse()
                return SearchResult(True, path, nodes_expanded, 
                                 time.time() - start_time_actual, current_cost)
            state = (current_pos, current_time)
            if state in visited:
                continue
            visited.add(state)
            # Explore neighbors
            for neighbor_pos, move_cost in self.env.get_neighbors(current_pos, current_time):
                next_time = current_time + 1
                new_cost = current_cost + move_cost
                neighbor_state = (neighbor_pos, next_time)
                if neighbor_state not in cost_map or new_cost < cost_map[neighbor_state]:
                    cost_map[neighbor_state] = new_cost
                    parent_map[neighbor_state] = (current_pos, current_time)
                    heapq.heappush(pq, (new_cost, neighbor_pos, next_time, None))
        return SearchResult(False, [], nodes_expanded, time.time() - start_time_actual)

class AStar(SearchAlgorithm):
    """A* search with admissible heuristic."""
    
    def __init__(self, environment: GridEnvironment, heuristic_func: Callable = None):
        super().__init__(environment)
        self.heuristic_func = heuristic_func or self.manhattan_distance
    
    def manhattan_distance(self, pos1: Position, pos2: Position) -> float:
        """Manhattan distance heuristic (admissible for 4-connected grid)."""
        return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y)
    
    def euclidean_distance(self, pos1: Position, pos2: Position) -> float:
        """Euclidean distance heuristic (admissible but less informed)."""
        return ((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2) ** 0.5
    
    def diagonal_distance(self, pos1: Position, pos2: Position) -> float:
        """Diagonal distance heuristic (admissible for 8-connected grid)."""
        dx = abs(pos1.x - pos2.x)
        dy = abs(pos1.y - pos2.y)
        return max(dx, dy) + (1.414 - 1) * min(dx, dy)
    
    def search(self, start: Position, goal: Position, 
               start_time: int = 0) -> SearchResult:
        start_time_actual = time.time()
        nodes_expanded = 0
        if start == goal:
            return SearchResult(True, [start], 0, time.time() - start_time_actual, 0.0)
        # Priority queue: (f_cost, g_cost, position, time, parent)
        pq = [(self.heuristic_func(start, goal), 0, start, start_time, None)]
        visited = set()
        g_cost_map = {(start, start_time): 0}
        parent_map = {}
        while pq:
            f_cost, g_cost, current_pos, current_time, parent = heapq.heappop(pq)
            nodes_expanded += 1
            if current_pos == goal:
                # Reconstruct path
                path = [goal]
                current_state = (current_pos, current_time)
                while current_state in parent_map:
                    current_state = parent_map[current_state]
                    path.append(current_state[0])
                path.reverse()
                return SearchResult(True, path, nodes_expanded, 
                                 time.time() - start_time_actual, g_cost)
            state = (current_pos, current_time)
            if state in visited:
                continue
            visited.add(state)
            # Explore neighbors
            for neighbor_pos, move_cost in self.env.get_neighbors(current_pos, current_time):
                next_time = current_time + 1
                new_g_cost = g_cost + move_cost
                h_cost = self.heuristic_func(neighbor_pos, goal)
                f_cost = new_g_cost + h_cost
                neighbor_state = (neighbor_pos, next_time)
                if neighbor_state not in g_cost_map or new_g_cost < g_cost_map[neighbor_state]:
                    g_cost_map[neighbor_state] = new_g_cost
                    parent_map[neighbor_state] = (current_pos, current_time)
                    heapq.heappush(pq, (f_cost, new_g_cost, neighbor_pos, next_time, None))
        return SearchResult(False, [], nodes_expanded, time.time() - start_time_actual)

class HillClimbingSearch(SearchAlgorithm):
    """Hill-climbing with random restarts for local search."""
    
    def __init__(self, environment: GridEnvironment, max_restarts: int = 20, 
                 max_steps: int = 2000, random_walk_prob: float = 0.1):
        super().__init__(environment)
        self.max_restarts = max_restarts
        self.max_steps = max_steps
        self.random_walk_prob = random_walk_prob
    
    def search(self, start: Position, goal: Position, 
               start_time: int = 0) -> SearchResult:
        start_time_actual = time.time()
        best_result = SearchResult(False, [], 0, 0, float('inf'))
        for restart in range(self.max_restarts):
            result = self._hill_climb(start, goal, start_time)
            if result.success and result.cost < best_result.cost:
                best_result = result
            # Early termination if we found a good solution
            if best_result.success and best_result.cost <= 1.1 * self._manhattan_distance(start, goal):
                break
        best_result.time_taken = time.time() - start_time_actual
        return best_result

    def _manhattan_distance(self, pos1: Position, pos2: Position) -> float:
        """Manhattan distance heuristic."""
        return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y)
    
    def _hill_climb(self, start: Position, goal: Position, 
                   start_time: int) -> SearchResult:
        """Perform single hill-climbing run with random walk."""
        current_pos = start
        current_time = start_time
        path = [start]
        cost = 0
        nodes_expanded = 0
        stuck_count = 0
        max_stuck = 50
        for step in range(self.max_steps):
            nodes_expanded += 1
            if current_pos == goal:
                return SearchResult(True, path, nodes_expanded, 0, cost)
            
            # Get all possible next moves
            neighbors = self.env.get_neighbors(current_pos, current_time)
            if not neighbors:
                break
            
            # Choose next move
            if random.random() < self.random_walk_prob:
                # Random walk to escape local optima
                next_pos, move_cost = random.choice(neighbors)
            else:
                # Greedy choice
                best_neighbor = None
                best_cost = float('inf')
                
                for neighbor_pos, move_cost in neighbors:
                    # Heuristic: distance to goal
                    h_cost = self._manhattan_distance(neighbor_pos, goal)
                    total_cost = cost + move_cost + h_cost
                    if total_cost < best_cost:
                        best_cost = total_cost
                        best_neighbor = (neighbor_pos, move_cost)
                if best_neighbor is None:
                    break
                next_pos, move_cost = best_neighbor
            
            # Check if we're making progress
            old_distance = self._manhattan_distance(current_pos, goal)
            new_distance = self._manhattan_distance(next_pos, goal)
            if new_distance >= old_distance:
                stuck_count += 1
                if stuck_count > max_stuck:
                    # Try random restart
                    break
            else:
                stuck_count = 0
            # Move to next position
            current_pos = next_pos
            current_time += 1
            path.append(current_pos)
            cost += move_cost
        return SearchResult(False, path, nodes_expanded, 0, cost)

class SimulatedAnnealingSearch(SearchAlgorithm):
    """Simulated annealing for local search."""
    def __init__(self, environment: GridEnvironment, initial_temp: float = 50.0,
                 cooling_rate: float = 0.98, max_iterations: int = 2000, 
                 min_temp: float = 0.1):
        super().__init__(environment)
        self.initial_temp = initial_temp
        self.cooling_rate = cooling_rate
        self.max_iterations = max_iterations
        self.min_temp = min_temp
    
    def search(self, start: Position, goal: Position, 
               start_time: int = 0) -> SearchResult:
        start_time_actual = time.time()
        nodes_expanded = 0
        if start == goal:
            return SearchResult(True, [start], 0, time.time() - start_time_actual, 0.0)
        # Try multiple runs with different initial paths
        best_result = SearchResult(False, [], 0, 0, float('inf'))
        for run in range(5):  # Multiple runs for better results
            result = self._simulated_annealing_run(start, goal, start_time, nodes_expanded)
            nodes_expanded += result.nodes_expanded
            if result.success and result.cost < best_result.cost:
                best_result = result
        best_result.time_taken = time.time() - start_time_actual
        best_result.nodes_expanded = nodes_expanded
        return best_result
    
    def _simulated_annealing_run(self, start: Position, goal: Position, 
                                start_time: int, base_nodes: int) -> SearchResult:
        """Single simulated annealing run."""
        # Generate initial path using A* if possible, otherwise random
        current_path = self._generate_initial_path(start, goal, start_time)
        current_cost = self._calculate_path_cost(current_path)
        best_path = current_path.copy()
        best_cost = current_cost
        temperature = self.initial_temp
        nodes_expanded = 0
        for iteration in range(self.max_iterations):
            nodes_expanded += 1
            # Generate neighbor path
            neighbor_path = self._generate_neighbor_path(current_path, start_time)
            neighbor_cost = self._calculate_path_cost(neighbor_path)
            # Accept or reject based on temperature
            if neighbor_cost < current_cost or random.random() < self._acceptance_probability(
                current_cost, neighbor_cost, temperature):
                current_path = neighbor_path
                current_cost = neighbor_cost
                if current_cost < best_cost:
                    best_path = current_path.copy()
                    best_cost = current_cost
            temperature *= self.cooling_rate
            if temperature < self.min_temp:
                break
        success = best_path[-1] == goal
        return SearchResult(success, best_path, nodes_expanded, 0, best_cost)
    
    def _generate_initial_path(self, start: Position, goal: Position, 
                              start_time: int) -> List[Position]:
        """Generate initial path using A* or random walk."""
        # Try A* first
        try:
            astar = AStar(self.env)
            result = astar.search(start, goal, start_time)
            if result.success:
                return result.path
        except:
            pass
        # Fall back to random walk
        return self._generate_random_path(start, goal, start_time)
    
    def _generate_random_path(self, start: Position, goal: Position, 
                            start_time: int) -> List[Position]:
        """Generate a random valid path from start to goal."""
        path = [start]
        current_pos = start
        current_time = start_time
        max_length = 100  # Prevent infinite loops
        for _ in range(max_length):
            if current_pos == goal:
                break
            neighbors = self.env.get_neighbors(current_pos, current_time)
            if not neighbors:
                break
            
            # Choose random neighbor
            next_pos, _ = random.choice(neighbors)
            current_pos = next_pos
            current_time += 1
            path.append(current_pos)
        return path
    
    def _generate_neighbor_path(self, path: List[Position], 
                              start_time: int) -> List[Position]:
        """Generate a neighbor path by making small modifications."""
        if len(path) < 2:
            return path.copy()
        # Randomly modify a portion of the path
        new_path = path.copy()
        start_idx = random.randint(0, len(path) - 2)
        end_idx = random.randint(start_idx + 1, len(path))
        
        # Try to find alternative path for the selected segment
        if start_idx > 0 and end_idx < len(path):
            segment_start = new_path[start_idx]
            segment_goal = new_path[end_idx]
            # Use A* to find alternative path for this segment
            a_star = AStar(self.env)
            result = a_star.search(segment_start, segment_goal, start_time + start_idx)
            if result.success and len(result.path) > 1:
                # Replace the segment
                new_path = new_path[:start_idx] + result.path[1:] + new_path[end_idx+1:]
        return new_path
    
    def _calculate_path_cost(self, path: List[Position]) -> float:
        """Calculate total cost of a path."""
        if len(path) < 2:
            return 0.0
        total_cost = 0.0
        for i in range(1, len(path)):
            total_cost += self.env.get_movement_cost(path[i].x, path[i].y)
        return total_cost
    
    def _acceptance_probability(self, current_cost: float, new_cost: float, 
                              temperature: float) -> float:
        """Calculate probability of accepting a worse solution."""
        if new_cost < current_cost:
            return 1.0
        return np.exp(-(new_cost - current_cost) / temperature)

def compare_algorithms(environment: GridEnvironment, start: Position, goal: Position,
                      start_time: int = 0) -> Dict[str, SearchResult]:
    """Compare all search algorithms on the same problem."""
    algorithms = {
        'BFS': BFS(environment),
        'Uniform Cost': UniformCostSearch(environment),
        'A* Manhattan': AStar(environment, AStar(environment).manhattan_distance),
        'A* Euclidean': AStar(environment, AStar(environment).euclidean_distance),
        'Hill Climbing': HillClimbingSearch(environment),
        'Simulated Annealing': SimulatedAnnealingSearch(environment)
    }
    results = {}
    for name, algorithm in algorithms.items():
        print(f"Running {name}...")
        result = algorithm.search(start, goal, start_time)
        results[name] = result
        if result.success:
            print(f"  Success: {len(result.path)} steps, cost: {result.cost:.2f}, "
                  f"nodes: {result.nodes_expanded}, time: {result.time_taken:.4f}s")
        else:
            print(f"  Failed: nodes: {result.nodes_expanded}, time: {result.time_taken:.4f}s")
    return results

if __name__ == "__main__":
    # Test the algorithms
    from environment import create_test_environment_small
    env = create_test_environment_small()
    start = Position(0, 0)
    goal = Position(9, 9)
    print("Testing search algorithms:")
    results = compare_algorithms(env, start, goal)
