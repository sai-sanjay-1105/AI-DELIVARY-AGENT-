import time
import logging
import json
from typing import List, Dict, Any
from datetime import datetime
from environment import GridEnvironment, Position, create_test_environment_dynamic
from delivery_agent import DeliveryAgent, PlanningStrategy, DeliveryTask

class DynamicReplanningDemo:
    """Demonstration of dynamic replanning capabilities."""
    def __init__(self, log_file: str = "dynamic_replanning_demo.log"):
        self.log_file = log_file
        self.setup_logging()
        self.env = create_test_environment_dynamic()
        self.agent = DeliveryAgent(self.env, Position(0, 0), initial_fuel=100.0)
        self.replanning_events = []
        
    def setup_logging(self):
        """Setup detailed logging for the demo."""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(self.log_file),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        
    def create_demo_tasks(self) -> List[DeliveryTask]:
        """Create demonstration tasks."""
        tasks = [
            DeliveryTask("urgent_package", Position(2, 2), Position(12, 12), priority=3),
            DeliveryTask("standard_package", Position(4, 4), Position(10, 10), priority=2),
            DeliveryTask("low_priority", Position(6, 6), Position(8, 8), priority=1),
        ]
        return tasks
    
    def log_environment_state(self, step: int):
        """Log current environment state."""
        dynamic_obstacles = self.env.get_dynamic_obstacles_at_time(self.env.current_time)
        self.logger.info(f"Step {step}: Environment state")
        self.logger.info(f"  Current time: {self.env.current_time}")
        self.logger.info(f"  Dynamic obstacles: {len(dynamic_obstacles)} at positions {list(dynamic_obstacles)}")
        self.logger.info(f"  Agent position: {self.agent.state.position}")
        self.logger.info(f"  Agent fuel: {self.agent.state.fuel:.2f}")
        self.logger.info(f"  Carrying packages: {self.agent.state.carrying_packages}")
        self.logger.info(f"  Completed deliveries: {self.agent.state.completed_deliveries}")
        if self.agent.current_path:
            remaining_path = self.agent.current_path[self.agent.path_index:]
            self.logger.info(f"  Planned path: {len(remaining_path)} steps remaining")
            if len(remaining_path) <= 5:
                self.logger.info(f"  Next steps: {remaining_path}")
            else:
                self.logger.info(f"  Next 5 steps: {remaining_path[:5]}")
    
    def detect_replanning_event(self, step: int) -> bool:
        """Detect if a replanning event occurred."""
        if self.agent.stats['replanning_events'] > len(self.replanning_events):
            event = {
                'step': step,
                'time': datetime.now().isoformat(),
                'agent_position': (self.agent.state.position.x, self.agent.state.position.y),
                'environment_time': self.env.current_time,
                'replanning_count': self.agent.stats['replanning_events'],
                'fuel_remaining': self.agent.state.fuel,
                'dynamic_obstacles': [
                    (pos.x, pos.y) for pos in self.env.get_dynamic_obstacles_at_time(self.env.current_time)
                ]
            }
            self.replanning_events.append(event)
            self.logger.warning(f"REPLANNING EVENT DETECTED at step {step}!")
            self.logger.warning(f"  Agent was blocked by dynamic obstacle")
            self.logger.warning(f"  Agent position: {self.agent.state.position}")
            self.logger.warning(f"  Dynamic obstacles: {event['dynamic_obstacles']}")
            self.logger.warning(f"  Total replanning events: {self.agent.stats['replanning_events']}")
            return True
        return False
    
    def print_environment_grid(self, step: int):
        """Print a visual representation of the environment."""
        print(f"\n=== Step {step} - Environment Grid ===")
        symbols = {
            'road': '.',
            'grass': 'g', 
            'water': '~',
            'mountain': '^',
            'building': '#',
            'agent': 'A',
            'dynamic_obstacle': 'O',
            'pickup': 'P',
            'delivery': 'D'
        }
        # Get dynamic obstacles
        dynamic_obstacles = self.env.get_dynamic_obstacles_at_time(self.env.current_time)
        
        # Get pickup and delivery locations
        pickup_locations = {task.pickup_location for task in self.agent.tasks 
                           if task.package_id not in self.agent.state.completed_deliveries}
        delivery_locations = {task.delivery_location for task in self.agent.tasks 
                             if task.package_id in self.agent.state.carrying_packages
                             }
        for y in range(self.env.height):
            row = ""
            for x in range(self.env.width):
                pos = Position(x, y)
                if pos == self.agent.state.position:
                    row += symbols['agent']
                elif pos in dynamic_obstacles:
                    row += symbols['dynamic_obstacle']
                elif pos in pickup_locations:
                    row += symbols['pickup']
                elif pos in delivery_locations:
                    row += symbols['delivery']
                else:
                    terrain = self.env.grid[y][x]
                    if terrain.value == 1:  # ROAD
                        row += symbols['road']
                    elif terrain.value == 2:  # GRASS
                        row += symbols['grass']
                    elif terrain.value == 3:  # WATER
                        row += symbols['water']
                    elif terrain.value == 4:  # MOUNTAIN
                        row += symbols['mountain']
                    elif terrain.value == 5:  # BUILDING
                        row += symbols['building']
                    else:
                        row += '?'
            print(row)
        print(f"Legend: A=Agent, O=Moving Obstacle, P=Pickup, D=Delivery, .=Road, g=Grass, ~=Water, ^=Mountain, #=Building")
    
    def run_demo(self, max_steps: int = 50, strategy: PlanningStrategy = PlanningStrategy.ASTAR_MANHATTAN):
        """Run the dynamic replanning demonstration."""
        self.logger.info("=" * 80)
        self.logger.info("STARTING DYNAMIC REPLANNING DEMONSTRATION")
        self.logger.info("=" * 80)
        
        # Setup
        self.agent.set_planning_strategy(strategy)
        tasks = self.create_demo_tasks()
        for task in tasks:
            self.agent.add_delivery_task(task)
        self.logger.info(f"Demo Configuration:")
        self.logger.info(f"  Strategy: {strategy.value}")
        self.logger.info(f"  Max steps: {max_steps}")
        self.logger.info(f"  Environment: {self.env.width}x{self.env.height}")
        self.logger.info(f"  Dynamic obstacles: {len(self.env.dynamic_obstacles)}")
        self.logger.info(f"  Tasks: {len(tasks)}")
        for i, task in enumerate(tasks):
            self.logger.info(f"    Task {i+1}: {task.package_id} from {task.pickup_location} to {task.delivery_location} (priority: {task.priority})")
        
        # Initial state
        self.print_environment_grid(0)
        self.log_environment_state(0)
        
        # Run simulation
        step = 0
        while step < max_steps and self.agent.state.fuel > 0:
            step += 1
            
            # Check if we need to plan
            if not self.agent.current_path or self.agent.path_index >= len(self.agent.current_path):
                if not self.agent.plan_next_action():
                    self.logger.info("No more tasks to complete!")
                    break
            
            # Detect replanning events
            replanning_detected = self.detect_replanning_event(step)
            
            # Execute next action
            if not self.agent.execute_next_action():
                self.logger.warning("Failed to execute action!")
                break
            
            # Advance environment time
            self.env.advance_time()
            
            # Log state every 5 steps or on replanning events
            if step % 5 == 0 or replanning_detected:
                self.print_environment_grid(step)
                self.log_environment_state(step)
            
            # Small delay for visualization
            time.sleep(0.1)
        
        # Final results
        self.print_final_results(step)
        self.save_demo_results()
        return {
            'steps_completed': step,
            'replanning_events': self.replanning_events,
            'agent_stats': self.agent.stats,
            'final_state': self.agent.get_status()
        }
    def print_final_results(self, final_step: int):
        """Print final demonstration results."""
        print("\n" + "=" * 80)
        print("DYNAMIC REPLANNING DEMONSTRATION COMPLETED")
        print("=" * 80)
        print(f"Simulation completed after {final_step} steps")
        print(f"Final agent position: {self.agent.state.position}")
        print(f"Final fuel remaining: {self.agent.state.fuel:.2f}")
        print(f"Deliveries completed: {self.agent.stats['deliveries_completed']}")
        print(f"Total distance traveled: {self.agent.stats['total_distance_traveled']:.2f}")
        print(f"Total fuel consumed: {self.agent.stats['total_fuel_consumed']:.2f}")
        print(f"Total planning time: {self.agent.stats['total_planning_time']:.4f}s")
        print(f"Replanning events: {self.agent.stats['replanning_events']}")
        if self.replanning_events:
            print(f"\nReplanning Event Details:")
            for i, event in enumerate(self.replanning_events):
                print(f"  Event {i+1}: Step {event['step']}, Position {event['agent_position']}, "
                      f"Obstacles {event['dynamic_obstacles']}")
        else:
            print("\nNo replanning events occurred - agent completed tasks without obstacles blocking the path.")
    
    def save_demo_results(self):
        """Save demonstration results to file."""
        results = {
            'demo_timestamp': datetime.now().isoformat(),
            'environment_config': {
                'width': self.env.width,
                'height': self.env.height,
                'dynamic_obstacles': len(self.env.dynamic_obstacles)
            },
            'agent_config': {
                'strategy': self.agent.current_strategy.value,
                'initial_fuel': self.agent.initial_fuel,
                'initial_position': (self.agent.initial_position.x, self.agent.initial_position.y)
            },
            'replanning_events': self.replanning_events,
            'final_stats': self.agent.stats,
            'final_state': self.agent.get_status()
        }
        with open('dynamic_replanning_results.json', 'w') as f:
            json.dump(results, f, indent=2)
        
        self.logger.info("Demo results saved to dynamic_replanning_results.json")

def run_strategy_comparison_demo():
    """Run demonstration comparing different strategies on dynamic replanning."""
    print("=" * 80)
    print("STRATEGY COMPARISON FOR DYNAMIC REPLANNING")
    print("=" * 80)
    strategies = [
        PlanningStrategy.ASTAR_MANHATTAN,
        PlanningStrategy.ASTAR_EUCLIDEAN,
        PlanningStrategy.HILL_CLIMBING,
        PlanningStrategy.SIMULATED_ANNEALING
    ]
    results = {}
    for strategy in strategies:
        print(f"\nTesting {strategy.value}...")
        demo = DynamicReplanningDemo(f"demo_{strategy.value.lower().replace(' ', '_').replace('*', 'star')}.log")
        result = demo.run_demo(max_steps=30, strategy=strategy)
        results[strategy.value] = result
    
    # Print comparison
    print("\n" + "=" * 80)
    print("STRATEGY COMPARISON RESULTS")
    print("=" * 80)
    print(f"{'Strategy':<25} {'Steps':<8} {'Deliveries':<12} {'Replanning':<12} {'Fuel Used':<12}")
    print("-" * 80)
    for strategy_name, result in results.items():
        steps = result['steps_completed']
        deliveries = result['agent_stats']['deliveries_completed']
        replanning = result['agent_stats']['replanning_events']
        fuel_used = result['agent_stats']['total_fuel_consumed']
        print(f"{strategy_name:<25} {steps:<8} {deliveries:<12} {replanning:<12} {fuel_used:<12.2f}")
    return results

if __name__ == "__main__":
    # Run single strategy demo
    print("Running single strategy dynamic replanning demo...")
    demo = DynamicReplanningDemo()
    demo.run_demo(max_steps=50, strategy=PlanningStrategy.ASTAR_MANHATTAN)
    # Run strategy comparison
    print("\n" + "=" * 80)
    print("Running strategy comparison demo...")
    run_strategy_comparison_demo()
