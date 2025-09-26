
import time
import functools
import weakref
from typing import Dict, List, Tuple, Optional, Any, Set
from collections import defaultdict, deque
import heapq
import numpy as np
from environment import GridEnvironment, Position, TerrainType
from search_algorithms import SearchAlgorithm, SearchResult
from delivery_agent import DeliveryAgent, DeliveryTask, PlanningStrategy

class PathCache:
    """Intelligent path caching system for repeated queries."""
    def __init__(self, max_size: int = 1000):

        self.cache: Dict[Tuple[Position, Position, int, str], SearchResult] = {}
        self.access_times: Dict[Tuple[Position, Position, int, str], float] = {}
        self.max_size = max_size
        self.hits = 0
        self.misses = 0

    def get(self, start: Position, goal: Position, time: int, algorithm: str) -> Optional[SearchResult]:
        """Get cached path if available."""
        key = (start, goal, time, algorithm)
        if key in self.cache:
            self.access_times[key] = time.time()
            self.hits += 1
            return self.cache[key]
        self.misses += 1
        return None
    
    def put(self, start: Position, goal: Position, time: int, algorithm: str, result: SearchResult):
        """Cache a path result."""
        key = (start, goal, time, algorithm)
        if len(self.cache) >= self.max_size:
            self._evict_oldest()
        self.cache[key] = result
        self.access_times[key] = time.time()

    def _evict_oldest(self):
        """Remove the least recently used entry."""
        if not self.access_times:
            return
        oldest_key = min(self.access_times.keys(), key=lambda k: self.access_times[k])
        del self.cache[oldest_key]
        del self.access_times[oldest_key]

    def clear(self):
        """Clear the cache."""
        self.cache.clear()
        self.access_times.clear()
        self.hits = 0
        self.misses = 0

    def get_stats(self) -> Dict[str, Any]:
        """Get cache statistics."""
        total_requests = self.hits + self.misses
        hit_rate = self.hits / total_requests if total_requests > 0 else 0
        return {
            'cache_size': len(self.cache),
            'max_size': self.max_size,
            'hits': self.hits,
            'misses': self.misses,
            'hit_rate': hit_rate
        }
    
class OptimizedAStar(SearchAlgorithm):
    """Optimized A* implementation with caching and early termination."""
    def __init__(self, environment: GridEnvironment, heuristic_func=None, use_cache: bool = True):
        super().__init__(environment)
        self.env = environment
        self.heuristic_func = heuristic_func or self.manhattan_distance
        self.use_cache = use_cache
        self.cache = PathCache() if use_cache else None
        self.max_nodes = 10000
        self.time_limit = 5.0
        self.early_termination_threshold = 1.1

    def manhattan_distance(self, pos1: Position, pos2: Position) -> float:
        """Manhattan distance heuristic."""
        return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y)
    
    def search(self, start: Position, goal: Position, start_time: int = 0) -> SearchResult:
        """Optimized A* search with caching and early termination."""
        start_time_actual = time.time()
        if self.cache:
            cached_result = self.cache.get(start, goal, start_time, "astar")
            if cached_result:
                return cached_result
        if start == goal:
            result = SearchResult(True, [start], 0, time.time() - start_time_actual, 0.0)
            if self.cache:
                self.cache.put(start, goal, start_time, "astar", result)
            return result
        result = self._optimized_search(start, goal, start_time, start_time_actual)
        if self.cache and result.success:
            self.cache.put(start, goal, start_time, "astar", result)
        return result
    def _optimized_search(self, start: Position, goal: Position, 
                            start_time: int, start_time_actual: float) -> SearchResult:
        """Optimized A* search implementation."""
        nodes_expanded = 0
        pq = [(self.heuristic_func(start, goal), 0, start, start_time, None)]
        visited = set()
        g_cost_map = {(start, start_time): 0}
        parent_map = {}
        best_heuristic = self.heuristic_func(start, goal)
        while pq and nodes_expanded < self.max_nodes:
            if time.time() - start_time_actual > self.time_limit:
                break
            f_cost, g_cost, current_pos, current_time, parent = heapq.heappop(pq)
            nodes_expanded += 1
            if current_pos == goal:
                path = self._reconstruct_path(current_pos, current_time, parent_map)
                return SearchResult(True, path, nodes_expanded, 
                                 time.time() - start_time_actual, g_cost)
            state = (current_pos, current_time)
            if state in visited:
                continue
            visited.add(state)
            for neighbor_pos, move_cost in self.env.get_neighbors(current_pos, current_time):
                next_time = current_time + 1
                new_g_cost = g_cost + move_cost
                h_cost = self.heuristic_func(neighbor_pos, goal)
                f_cost = new_g_cost + h_cost
                neighbor_state = (neighbor_pos, next_time)
                if (h_cost < best_heuristic * self.early_termination_threshold and 
                    neighbor_pos == goal):
                    path = self._reconstruct_path(neighbor_pos, next_time, parent_map)
                    return SearchResult(True, path, nodes_expanded, 
                                     time.time() - start_time_actual, new_g_cost)
                if neighbor_state not in g_cost_map or new_g_cost < g_cost_map[neighbor_state]:
                    g_cost_map[neighbor_state] = new_g_cost
                    parent_map[neighbor_state] = (current_pos, current_time)
                    heapq.heappush(pq, (f_cost, new_g_cost, neighbor_pos, next_time, None))
        return SearchResult(False, [], nodes_expanded, time.time() - start_time_actual, 0.0)
    def _reconstruct_path(self, goal_pos: Position, goal_time: int, 
                            parent_map: Dict) -> List[Position]:
        """Reconstruct path from parent map."""
        path = [goal_pos]
        current_state = (goal_pos, goal_time)
        while current_state in parent_map:
            current_state = parent_map[current_state]
            path.append(current_state[0])
        path.reverse()
        return path
    
class HierarchicalPlanner:
    """Hierarchical planning for large maps using abstraction."""
    def __init__(self, environment: GridEnvironment, cluster_size: int = 10):
        self.env = environment
        self.cluster_size = cluster_size
        self.clusters = self._create_clusters()
        self.cluster_graph = self._build_cluster_graph()

    def _create_clusters(self) -> Dict[Tuple[int, int], List[Position]]:
        """Create clusters of positions for hierarchical planning."""
        clusters = {}
        for y in range(0, self.env.height, self.cluster_size):
            for x in range(0, self.env.width, self.cluster_size):
                cluster_id = (x // self.cluster_size, y // self.cluster_size)
                positions = []
                for dy in range(min(self.cluster_size, self.env.height - y)):
                    for dx in range(min(self.cluster_size, self.env.width - x)):
                        pos = Position(x + dx, y + dy)
                        if self.env.is_passable(pos.x, pos.y):
                            positions.append(pos)
                if positions:
                    clusters[cluster_id] = positions
        return clusters
    
    def _build_cluster_graph(self) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
        """Build graph of connected clusters."""
        graph = defaultdict(list)
        for cluster_id in self.clusters:
            x, y = cluster_id
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                neighbor_id = (x + dx, y + dy)
                if neighbor_id in self.clusters:
                    graph[cluster_id].append(neighbor_id)
        return dict(graph)
    
    def plan_path(self, start: Position, goal: Position, start_time: int = 0) -> SearchResult:
        """Plan path using hierarchical approach."""
        start_time_actual = time.time()
        start_cluster = self._find_cluster(start)
        goal_cluster = self._find_cluster(goal)
        if not start_cluster or not goal_cluster:
            return SearchResult(False, [], 0, time.time() - start_time_actual, 0.0)
        if start_cluster == goal_cluster:
            return self._plan_within_cluster(start, goal, start_time)
        cluster_path = self._plan_cluster_path(start_cluster, goal_cluster)
        if not cluster_path:
            return SearchResult(False, [], 0, time.time() - start_time_actual, 0.0)
        detailed_path = self._plan_detailed_path(start, goal, cluster_path, start_time)
        return SearchResult(True, detailed_path, 0, time.time() - start_time_actual, len(detailed_path))
    
    def _find_cluster(self, pos: Position) -> Optional[Tuple[int, int]]:
        """Find cluster containing given position."""
        cluster_x = pos.x // self.cluster_size
        cluster_y = pos.y // self.cluster_size
        cluster_id = (cluster_x, cluster_y)
        return cluster_id if cluster_id in self.clusters else None
    
    def _plan_within_cluster(self, start: Position, goal: Position, start_time: int) -> SearchResult:
        """Plan path within a single cluster."""
        astar = OptimizedAStar(self.env, use_cache=False)
        return astar.search(start, goal, start_time)
    
    def _plan_cluster_path(self, start_cluster: Tuple[int, int], goal_cluster: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Plan path through clusters using BFS."""
        if start_cluster == goal_cluster:
            return [start_cluster]
        queue = deque([(start_cluster, [start_cluster])])
        visited = {start_cluster}
        while queue:
            current_cluster, path = queue.popleft()
            if current_cluster == goal_cluster:
                return path
            for neighbor in self.cluster_graph.get(current_cluster, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return []
    
    def _plan_detailed_path(self, start: Position, goal: Position, cluster_path: List[Tuple[int, int]], start_time: int) -> List[Position]:
        """Plan detailed path through cluster sequence."""
        if not cluster_path:
            return []
        detailed_path = [start]
        current_pos = start
        current_time = start_time
        for i in range(len(cluster_path) - 1):
            current_cluster = cluster_path[i]
            next_cluster = cluster_path[i + 1]
            transition_pos = self._find_transition_point(current_cluster, next_cluster, current_pos)
            if not transition_pos:
                break
            segment_path = self._plan_within_cluster(current_pos, transition_pos, current_time)
            if not segment_path.success:
                break
            detailed_path.extend(segment_path.path[1:])
            current_pos = transition_pos
            current_time += len(segment_path.path) - 1
        if current_pos != goal:
            final_segment = self._plan_within_cluster(current_pos, goal, current_time)
            if final_segment.success:
                detailed_path.extend(final_segment.path[1:])
        return detailed_path
    
    def _find_transition_point(self, from_cluster: Tuple[int, int], to_cluster: Tuple[int, int], current_pos: Position) -> Optional[Position]:
        """Find best transition point between clusters."""
        from_positions = self.clusters[from_cluster]
        to_positions = self.clusters[to_cluster]
        best_point = None
        best_distance = float('inf')
        for from_pos in from_positions:
            for to_pos in to_positions:
                if (abs(from_pos.x - to_pos.x) + abs(from_pos.y - to_pos.y) == 1):
                    distance = abs(from_pos.x - current_pos.x) + abs(from_pos.y - current_pos.y)
                    if distance < best_distance:
                        best_distance = distance
                        best_point = from_pos
        return best_point
    
class MemoryOptimizedAgent(DeliveryAgent):
    """Memory-optimized delivery agent for large-scale scenarios."""
    def __init__(self, environment: GridEnvironment, initial_position: Position, initial_fuel: float = 100.0, fuel_consumption_rate: float = 1.0, max_path_history: int = 100):
        super().__init__(environment, initial_position, initial_fuel, fuel_consumption_rate)
        self.max_path_history = max_path_history
        self.path_history = deque(maxlen=max_path_history)
        self.planning_cache = PathCache(max_size=500)
        self.optimized_algorithms = {
            PlanningStrategy.ASTAR_MANHATTAN: OptimizedAStar(environment, use_cache=True),
            PlanningStrategy.ASTAR_EUCLIDEAN: OptimizedAStar(environment, lambda p1, p2: ((p1.x-p2.x)**2 + (p1.y-p2.y)**2)**0.5, use_cache=True),
            PlanningStrategy.ASTAR_DIAGONAL: OptimizedAStar(environment, self._diagonal_distance, use_cache=True)
        }

    def _diagonal_distance(self, pos1: Position, pos2: Position) -> float:
        """Diagonal distance heuristic."""
        dx = abs(pos1.x - pos2.x)
        dy = abs(pos1.y - pos2.y)
        return max(dx, dy) + (1.414 - 1) * min(dx, dy)
    
    def _create_algorithm(self, strategy: PlanningStrategy) -> SearchAlgorithm:
        """Create optimized algorithm based on strategy."""
        if strategy in self.optimized_algorithms:
            return self.optimized_algorithms[strategy]
        return super()._create_algorithm(strategy)
    
    def plan_next_action(self) -> bool:
        """Optimized planning with caching."""
        if not self.planning_algorithm:
            self.planning_algorithm = self._create_algorithm(self.current_strategy)
        start_time = time.time()
        next_goal = self._determine_next_goal()
        if not next_goal:
            return False
        cache_key = (self.state.position, next_goal, getattr(self.state, "time", 0), self.current_strategy.value)
        cached_result = self.planning_cache.get(*cache_key)
        if cached_result:
            self.current_path = cached_result.path[1:]
            self.path_index = 0
            self.stats['total_planning_time'] += time.time() - start_time
            return True
        result = self.planning_algorithm.search(self.state.position, next_goal, getattr(self.state, "time", 0))
        planning_time = time.time() - start_time
        self.stats['total_planning_time'] += planning_time
        if result.success:
            self.current_path = result.path[1:]
            self.path_index = 0
            self.planning_cache.put(*cache_key, result)
            self.path_history.append({'path': result.path.copy(), 'cost': result.cost, 'planning_time': planning_time, 'timestamp': getattr(self.state, "time", 0)})
            return True
        return False
    
    def get_optimization_stats(self) -> Dict[str, Any]:
        """Get optimization statistics."""
        stats = self.stats.copy()
        stats.update({
            'cache_stats': self.planning_cache.get_stats(),
            'path_history_size': len(self.path_history),
            'memory_usage_estimate': self._estimate_memory_usage()
        })
        return stats
    
    def _estimate_memory_usage(self) -> int:
        """Estimate memory usage in bytes."""
        base_memory = 1000
        path_memory = len(self.path_history) * 1000
        cache_memory = self.planning_cache.get_stats()['cache_size'] * 500
        return base_memory + path_memory + cache_memory
    
class PerformanceProfiler:
    """Performance profiler for identifying bottlenecks."""
    def __init__(self):
        self.profiles = defaultdict(list)
        self.active_profiles = {}

    def start_profile(self, name: str):
        """Start profiling a function or operation."""
        self.active_profiles[name] = time.time()

    def end_profile(self, name: str):
        """End profiling and record duration."""
        if name in self.active_profiles:
            duration = time.time() - self.active_profiles[name]
            self.profiles[name].append(duration)
            del self.active_profiles[name]

    def get_profile_stats(self) -> Dict[str, Dict[str, float]]:
        """Get profiling statistics."""
        stats = {}
        for name, durations in self.profiles.items():
            if durations:
                stats[name] = {
                    'count': len(durations),
                    'total_time': sum(durations),
                    'average_time': sum(durations) / len(durations),
                    'min_time': min(durations),
                    'max_time': max(durations)
                }
        return stats
    
    def clear_profiles(self):
        """Clear all profiling data."""
        self.profiles.clear()
        self.active_profiles.clear()

def optimize_for_large_maps(environment: GridEnvironment, max_size_threshold: int = 30) -> Dict[str, Any]:
    """Automatically optimize system for large maps."""
    optimizations = {
        'use_hierarchical_planning': environment.width > max_size_threshold or environment.height > max_size_threshold,
        'use_path_caching': True,
        'use_memory_optimization': True,
        'cluster_size': min(10, max(5, min(environment.width, environment.height) // 5)),
        'max_nodes_limit': 10000,
        'time_limit': 5.0
    }
    return optimizations

def benchmark_optimizations(environment: GridEnvironment, num_runs: int = 10) -> Dict[str, Any]:
    """Benchmark different optimization strategies."""
    from search_algorithms import AStar, compare_algorithms
    from delivery_agent import DeliveryAgent, PlanningStrategy, DeliveryTask
    results = {}
    start = Position(0, 0)
    goal = Position(min(environment.width-1, 20), min(environment.height-1, 20))
    print("Benchmarking standard A*...")
    standard_times = []
    standard_successes = []
    for _ in range(num_runs):
        astar = AStar(environment)
        start_time = time.time()
        result = astar.search(start, goal)
        standard_times.append(time.time() - start_time)
        standard_successes.append(result.success)
    results['standard_astar'] = {
        'average_time': sum(standard_times) / len(standard_times),
        'success_rate': sum(standard_successes) / len(standard_successes)
    }
    print("Benchmarking optimized A*...")
    optimized_times = []
    for _ in range(num_runs):
        opt_astar = OptimizedAStar(environment, use_cache=True)
        start_time = time.time()
        result = opt_astar.search(start, goal)
        optimized_times.append(time.time() - start_time)
    results['optimized_astar'] = {
        'average_time': sum(optimized_times) / len(optimized_times),
        'success_rate': 1.0 if all(optimized_times) else 0.0,
        'cache_stats': opt_astar.cache.get_stats() if opt_astar.cache else {}
    }
    if environment.width > 30 or environment.height > 30:
        print("Benchmarking hierarchical planning...")
        hierarchical_times = []
        for _ in range(num_runs):
            hier_planner = HierarchicalPlanner(environment)
            start_time = time.time()
            result = hier_planner.plan_path(start, goal)
            hierarchical_times.append(time.time() - start_time)
        results['hierarchical_planning'] = {
            'average_time': sum(hierarchical_times) / len(hierarchical_times),
            'success_rate': 1.0 if all(hierarchical_times) else 0.0
        }
    return results

if __name__ == "__main__":
    from environment import create_test_environment_large
    print("Testing optimization utilities...")
    cache = PathCache(max_size=10)
    print(f"Cache stats: {cache.get_stats()}")
    env = create_test_environment_large()
    opt_astar = OptimizedAStar(env, use_cache=True)
    start = Position(0, 0)
    goal = Position(49, 49)
    print("Testing optimized A* on large map...")
    result = opt_astar.search(start, goal)
    print(f"Result: {result.success}, Time: {result.time_taken:.4f}s, Nodes: {result.nodes_expanded}")
    print(f"Cache stats: {opt_astar.cache.get_stats()}")
    print("Testing hierarchical planning...")
    hier_planner = HierarchicalPlanner(env, cluster_size=10)
    result = hier_planner.plan_path(start, goal)
    print(f"Hierarchical result: {result.success}, Time: {result.time_taken:.4f}s")
    print("Optimization tests completed!")