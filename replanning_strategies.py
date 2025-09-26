import time
import random
from typing import List, Optional, Dict, Any, Tuple
from enum import Enum
from dataclasses import dataclass
from environment import GridEnvironment, Position
from delivery_agent import DeliveryAgent, DeliveryTask
from search_algorithms import SearchAlgorithm, AStar, BFS, UniformCostSearch

class ReplanningStrategy(Enum):
    """Available replanning strategies."""
    IMMEDIATE = "immediate"           # Replan immediately when path becomes invalid
    PREDICTIVE = "predictive"         # Predict obstacles and replan proactively
    ADAPTIVE = "adaptive"            # Adapt replanning frequency based on environment
    CONSERVATIVE = "conservative"     # Replan only when absolutely necessary
    AGGRESSIVE = "aggressive"        # Replan frequently to maintain optimal paths

@dataclass
class ReplanningEvent:
    """Represents a replanning event with context."""
    timestamp: int
    reason: str
    old_path_length: int
    new_path_length: int
    planning_time: float
    success: bool

class AdvancedReplanningAgent(DeliveryAgent):
    """Enhanced delivery agent with advanced replanning capabilities."""
    def __init__(self, environment: GridEnvironment, initial_position: Position,
                 initial_fuel: float = 100.0, fuel_consumption_rate: float = 1.0,
                 replanning_strategy: ReplanningStrategy = ReplanningStrategy.ADAPTIVE):
        super().__init__(environment, initial_position, initial_fuel, fuel_consumption_rate)
        self.replanning_strategy = replanning_strategy
        self.replanning_events: List[ReplanningEvent] = []
        self.last_replan_time = 0
        self.replanning_frequency = 1.0  # Adaptive frequency
        self.obstacle_prediction_window = 5  # Steps ahead to predict
        # Statistics
        self.stats.update({
            'predictive_replans': 0,
            'reactive_replans': 0,
            'failed_replans': 0,
            'average_replan_time': 0.0,
            'replanning_efficiency': 0.0
        })
    def set_replanning_strategy(self, strategy: ReplanningStrategy):
        """Set the replanning strategy."""
        self.replanning_strategy = strategy
        self.logger.info(f"Switched to replanning strategy: {strategy.value}")
    def should_replan(self) -> Tuple[bool, str]:
        """Determine if replanning is needed based on current strategy."""
        current_time = self.env.current_time
        # Check if path is completely invalid
        if not self._is_path_still_valid():
            return True, "Path blocked by obstacles"
        # Strategy-specific replanning logic
        if self.replanning_strategy == ReplanningStrategy.IMMEDIATE:
            return self._immediate_replanning()
        elif self.replanning_strategy == ReplanningStrategy.PREDICTIVE:
            return self._predictive_replanning()
        elif self.replanning_strategy == ReplanningStrategy.ADAPTIVE:
            return self._adaptive_replanning()
        elif self.replanning_strategy == ReplanningStrategy.CONSERVATIVE:
            return self._conservative_replanning()
        elif self.replanning_strategy == ReplanningStrategy.AGGRESSIVE:
            return self._aggressive_replanning()
        return False, "No replanning needed"
    
    def _immediate_replanning(self) -> Tuple[bool, str]:
        """Replan immediately when path becomes invalid."""
        if not self._is_path_still_valid():
            return True, "Path invalid - immediate replan"
        return False, "Path valid"
    
    def _predictive_replanning(self) -> Tuple[bool, str]:
        """Predict obstacles and replan proactively."""
        if not self.current_path or self.path_index >= len(self.current_path):
            return True, "No current path"

        # Check if obstacles will block path in the near future
        for i in range(self.path_index, min(len(self.current_path), 
                                          self.path_index + self.obstacle_prediction_window)):
            future_time = self.state.time + (i - self.path_index)
            pos = self.current_path[i]
            if not self.env.is_passable(pos.x, pos.y, future_time):
                return True, f"Obstacle predicted at step {i}"
        return False, "No obstacles predicted"
    
    def _adaptive_replanning(self) -> Tuple[bool, str]:
        """Adapt replanning frequency based on environment dynamics."""
        time_since_last_replan = self.env.current_time - self.last_replan_time
        # Increase frequency if recent replans were frequent
        recent_replans = [e for e in self.replanning_events 
                         if e.timestamp > self.env.current_time - 10]
        if len(recent_replans) > 3:
            self.replanning_frequency = min(2.0, self.replanning_frequency * 1.1)
        else:
            self.replanning_frequency = max(0.5, self.replanning_frequency * 0.95)
        # Replan based on adaptive frequency
        if time_since_last_replan >= 1 / self.replanning_frequency:
            if not self._is_path_still_valid():
                return True, "Adaptive replan - path invalid"
            # Check for better paths
            if self._has_better_path_available():
                return True, "Adaptive replan - better path available"
        return False, "Adaptive - no replan needed"
    
    def _conservative_replanning(self) -> Tuple[bool, str]:
        """Replan only when absolutely necessary."""
        if not self._is_path_still_valid():
            return True, "Conservative replan - path blocked"
        # Only replan if current path is significantly suboptimal
        if self.current_path and len(self.current_path) > 0:
            remaining_path = self.current_path[self.path_index:]
            if len(remaining_path) > 2 * self._manhattan_distance(self.state.position, 
                                                                 self._get_current_goal()):
                return True, "Conservative replan - path too long"
        return False, "Conservative - no replan needed"
    
    def _aggressive_replanning(self) -> Tuple[bool, str]:
        """Replan frequently to maintain optimal paths."""
        time_since_last_replan = self.env.current_time - self.last_replan_time
        if time_since_last_replan >= 2:  # Replan every 2 steps
            return True, "Aggressive replan - regular interval"
        if not self._is_path_still_valid():
            return True, "Aggressive replan - path invalid"
        return False, "Aggressive - no replan needed"

    def _has_better_path_available(self) -> bool:
        """Check if a significantly better path is available."""
        if not self.current_path or self.path_index >= len(self.current_path):
            return True
        # Quick A* search to find alternative path
        try:
            current_goal = self._get_current_goal()
            if not current_goal:
                return False
            astar = AStar(self.env)
            result = astar.search(self.state.position, current_goal, self.state.time)
            if result.success:
                current_remaining = len(self.current_path) - self.path_index
                new_path_length = len(result.path) - 1
                # Consider it better if it's at least 20% shorter
                return new_path_length < 0.8 * current_remaining
        except:
            pass
        
        return False
    
    def _get_current_goal(self) -> Optional[Position]:
        """Get the current goal position."""
        return self._determine_next_goal()
    
    def _manhattan_distance(self, pos1: Position, pos2: Position) -> float:
        """Manhattan distance between two positions."""
        return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y)
    
    def execute_next_action_with_replanning(self) -> bool:
        """Execute next action with intelligent replanning."""
        # Check if replanning is needed
        should_replan, reason = self.should_replan()
        if should_replan:
            return self._perform_replanning(reason)
        # Execute normal action
        return self.execute_next_action()
    
    def _perform_replanning(self, reason: str) -> bool:
        """Perform replanning and log the event."""
        start_time = time.time()
        old_path_length = len(self.current_path) - self.path_index if self.current_path else 0
        # Clear current path
        self.current_path = []
        self.path_index = 0
        # Plan new path
        success = self.plan_next_action()
        planning_time = time.time() - start_time
        new_path_length = len(self.current_path) if self.current_path else 0
        
        # Log replanning event
        event = ReplanningEvent(
            timestamp=self.env.current_time,
            reason=reason,
            old_path_length=old_path_length,
            new_path_length=new_path_length,
            planning_time=planning_time,
            success=success
        )
        self.replanning_events.append(event)
        self.last_replan_time = self.env.current_time
        # Update statistics
        self.stats['replanning_events'] += 1
        if "predicted" in reason.lower():
            self.stats['predictive_replans'] += 1
        else:
            self.stats['reactive_replans'] += 1
        
        if not success:
            self.stats['failed_replans'] += 1
        # Update average replan time
        total_time = sum(e.planning_time for e in self.replanning_events)
        self.stats['average_replan_time'] = total_time / len(self.replanning_events)
        # Calculate replanning efficiency
        successful_replans = len([e for e in self.replanning_events if e.success])
        self.stats['replanning_efficiency'] = successful_replans / len(self.replanning_events) if self.replanning_events else 0
        self.logger.info(f"Replanning: {reason} - Success: {success}, "
                        f"Time: {planning_time:.4f}s, "
                        f"Old: {old_path_length}, New: {new_path_length}")
        return success
    
    def get_replanning_statistics(self) -> Dict[str, Any]:
        """Get detailed replanning statistics."""
        if not self.replanning_events:
            return self.stats
        
        # Calculate additional metrics
        recent_events = [e for e in self.replanning_events if e.timestamp > self.env.current_time - 20]
        stats = self.stats.copy()
        stats.update({
            'total_replanning_events': len(self.replanning_events),
            'recent_replanning_events': len(recent_events),
            'average_path_improvement': self._calculate_path_improvement(),
            'replanning_reasons': self._get_replanning_reasons(),
            'planning_time_trend': self._get_planning_time_trend()
        })
        return stats
    
    def _calculate_path_improvement(self) -> float:
        """Calculate average path improvement from replanning."""
        improvements = []
        for i in range(1, len(self.replanning_events)):
            prev_event = self.replanning_events[i-1]
            curr_event = self.replanning_events[i]
            if prev_event.success and curr_event.success:
                improvement = (prev_event.old_path_length - curr_event.new_path_length) / prev_event.old_path_length
                improvements.append(improvement)
        return sum(improvements) / len(improvements) if improvements else 0.0
    
    def _get_replanning_reasons(self) -> Dict[str, int]:
        """Get frequency of different replanning reasons."""
        reasons = {}
        for event in self.replanning_events:
            reason_type = event.reason.split(' - ')[0]  # Get main reason
            reasons[reason_type] = reasons.get(reason_type, 0) + 1
        return reasons
    def _get_planning_time_trend(self) -> List[float]:
        """Get trend of planning times over recent events."""
        recent_events = self.replanning_events[-10:]  # Last 10 events
        return [event.planning_time for event in recent_events]

def compare_replanning_strategies(environment: GridEnvironment, 
                                tasks: List[DeliveryTask],
                                max_steps: int = 1000) -> Dict[str, Any]:
    """Compare different replanning strategies on the same scenario."""
    strategies = [
        ReplanningStrategy.IMMEDIATE,
        ReplanningStrategy.PREDICTIVE,
        ReplanningStrategy.ADAPTIVE,
        ReplanningStrategy.CONSERVATIVE,
        ReplanningStrategy.AGGRESSIVE
    ]
    results = {}
    for strategy in strategies:
        print(f"Testing {strategy.value} replanning strategy...")
        # Create agent with strategy
        agent = AdvancedReplanningAgent(environment, Position(0, 0), 
                                      initial_fuel=100.0, replanning_strategy=strategy)
        # Add tasks
        for task in tasks:
            agent.add_delivery_task(task)
        # Run simulation
        start_time = time.time()
        step = 0
        while step < max_steps and agent.state.fuel > 0:
            if not agent.execute_next_action_with_replanning():
                break
            step += 1
            environment.advance_time()
        
        # Collect results
        simulation_time = time.time() - start_time
        replanning_stats = agent.get_replanning_statistics()
        results[strategy.value] = {
            'deliveries_completed': agent.stats['deliveries_completed'],
            'simulation_steps': step,
            'total_fuel_consumed': agent.stats['total_fuel_consumed'],
            'simulation_time': simulation_time,
            'replanning_events': replanning_stats['total_replanning_events'],
            'predictive_replans': replanning_stats['predictive_replans'],
            'reactive_replans': replanning_stats['reactive_replans'],
            'failed_replans': replanning_stats['failed_replans'],
            'average_replan_time': replanning_stats['average_replan_time'],
            'replanning_efficiency': replanning_stats['replanning_efficiency'],
            'path_improvement': replanning_stats['average_path_improvement']
        }
        print(f"  Completed {agent.stats['deliveries_completed']} deliveries in {step} steps")
        print(f"  Replanning events: {replanning_stats['total_replanning_events']}")
        print(f"  Efficiency: {replanning_stats['replanning_efficiency']:.2f}")
    
    return results

if __name__ == "__main__":
    # Test replanning strategies
    from environment import create_test_environment_dynamic
    from delivery_agent import DeliveryTask
    env = create_test_environment_dynamic()
    tasks = [
        DeliveryTask("urgent", Position(1, 1), Position(13, 13), priority=3),
        DeliveryTask("standard", Position(2, 2), Position(12, 12), priority=2),
    ]
    print("Comparing replanning strategies...")
    results = compare_replanning_strategies(env, tasks)
    print("\nReplanning Strategy Comparison:")
    print("-" * 80)
    print(f"{'Strategy':<15} {'Deliveries':<12} {'Steps':<8} {'Replans':<10} {'Efficiency':<12}")
    print("-" * 80)
    for strategy, stats in results.items():
        print(f"{strategy:<15} {stats['deliveries_completed']:<12} {stats['simulation_steps']:<8} "
              f"{stats['replanning_events']:<10} {stats['replanning_efficiency']:<12.2f}")
