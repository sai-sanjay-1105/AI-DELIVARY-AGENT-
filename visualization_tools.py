import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from typing import List, Dict, Any, Optional, Tuple
import time
from datetime import datetime
from environment import GridEnvironment, Position, TerrainType
from delivery_agent import DeliveryAgent, DeliveryTask, PlanningStrategy
from search_algorithms import SearchResult

class AgentVisualizer:
    """Visualization tool for agent behavior and pathfinding."""
    def __init__(self, environment: GridEnvironment, agent: DeliveryAgent):
        self.env = environment
        self.agent = agent
        self.fig = None
        self.ax = None
        self.setup_colors()
        
    def setup_colors(self):
        """Setup color scheme for visualization."""
        self.colors = {
            'road': '#FFFFFF',
            'grass': '#90EE90',
            'water': '#87CEEB',
            'mountain': '#8B7355',
            'building': '#696969',
            'agent': '#FF0000',
            'dynamic_obstacle': '#FFA500',
            'pickup': '#00FF00',
            'delivery': '#0000FF',
            'path': '#FFD700',
            'visited': '#DDA0DD'
        }
    
    def create_environment_visualization(self, title: str = "Environment", 
                                       show_path: bool = False,
                                       show_dynamic_obstacles: bool = True) -> plt.Figure:
        """Create a static visualization of the environment."""
        fig, ax = plt.subplots(figsize=(12, 10))
        # Create grid visualization
        grid_colors = np.zeros((self.env.height, self.env.width, 3))
        for y in range(self.env.height):
            for x in range(self.env.width):
                terrain = self.env.grid[y][x]
                if terrain == TerrainType.ROAD:
                    color = [1, 1, 1]  # White
                elif terrain == TerrainType.GRASS:
                    color = [0.56, 0.93, 0.56]  # Light green
                elif terrain == TerrainType.WATER:
                    color = [0.53, 0.81, 0.92]  # Sky blue
                elif terrain == TerrainType.MOUNTAIN:
                    color = [0.55, 0.45, 0.33]  # Brown
                elif terrain == TerrainType.BUILDING:
                    color = [0.41, 0.41, 0.41]  # Dark gray
                else:
                    color = [0.5, 0.5, 0.5]  # Gray
                
                grid_colors[y, x] = color
        # Display the grid
        ax.imshow(grid_colors, origin='lower')
        # Add dynamic obstacles if enabled
        if show_dynamic_obstacles:
            dynamic_obstacles = self.env.get_dynamic_obstacles_at_time(self.env.current_time)
            for obstacle in dynamic_obstacles:
                ax.scatter(obstacle.x, obstacle.y, c='orange', s=200, marker='s', 
                          label='Moving Obstacle' if obstacle == list(dynamic_obstacles)[0] else "")
        # Add agent
        ax.scatter(self.agent.state.position.x, self.agent.state.position.y, 
                  c='red', s=300, marker='o', label='Agent')
        # Add pickup and delivery locations
        pickup_locations = set()
        delivery_locations = set()
        for task in self.agent.tasks:
            if task.package_id not in self.agent.state.completed_deliveries:
                pickup_locations.add(task.pickup_location)
            if task.package_id in self.agent.state.carrying_packages:
                delivery_locations.add(task.delivery_location)
        for pickup in pickup_locations:
            ax.scatter(pickup.x, pickup.y, c='green', s=200, marker='^', 
                      label='Pickup' if pickup == list(pickup_locations)[0] else "")
        for delivery in delivery_locations:
            ax.scatter(delivery.x, delivery.y, c='blue', s=200, marker='v', 
                      label='Delivery' if delivery == list(delivery_locations)[0] else "")
        # Add planned path if enabled
        if show_path and self.agent.current_path:
            path_x = [pos.x for pos in self.agent.current_path[self.agent.path_index:]]
            path_y = [pos.y for pos in self.agent.current_path[self.agent.path_index:]]
            ax.plot(path_x, path_y, 'gold', linewidth=3, alpha=0.7, label='Planned Path')
        # Customize plot
        ax.set_xlim(-0.5, self.env.width - 0.5)
        ax.set_ylim(-0.5, self.env.height - 0.5)
        ax.set_xticks(range(self.env.width))
        ax.set_yticks(range(self.env.height))
        ax.grid(True, alpha=0.3)
        ax.set_title(title, fontsize=16, fontweight='bold')
        ax.set_xlabel('X Position', fontsize=12)
        ax.set_ylabel('Y Position', fontsize=12)
        
        # Add legend
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(handles, labels, loc='upper right', bbox_to_anchor=(1.3, 1))
        
        # Add status text
        status_text = f"Step: {self.env.current_time}\n"
        status_text += f"Fuel: {self.agent.state.fuel:.1f}\n"
        status_text += f"Carrying: {len(self.agent.state.carrying_packages)}\n"
        status_text += f"Completed: {len(self.agent.state.completed_deliveries)}\n"
        status_text += f"Replanning: {self.agent.stats['replanning_events']}"
        ax.text(0.02, 0.98, status_text, transform=ax.transAxes, 
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        plt.tight_layout()
        return fig
    
    def create_path_comparison_plot(self, results: Dict[str, SearchResult], 
                                  start: Position, goal: Position) -> plt.Figure:
        """Create a plot comparing different pathfinding algorithms."""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        axes = axes.flatten()
        algorithms = list(results.keys())
        for i, (alg_name, result) in enumerate(results.items()):
            ax = axes[i]
            if i >= len(axes):
                break
            # Create base environment
            self._draw_environment_base(ax)
            
            # Draw path if successful
            if result.success and result.path:
                path_x = [pos.x for pos in result.path]
                path_y = [pos.y for pos in result.path]
                ax.plot(path_x, path_y, 'red', linewidth=3, alpha=0.8, label='Path')
                
                # Mark start and goal
                ax.scatter(start.x, start.y, c='green', s=300, marker='o', label='Start')
                ax.scatter(goal.x, goal.y, c='red', s=300, marker='s', label='Goal')
                
                # Add algorithm info
                info_text = f"{alg_name}\n"
                info_text += f"Success: {result.success}\n"
                info_text += f"Path Length: {len(result.path)}\n"
                info_text += f"Cost: {result.cost:.2f}\n"
                info_text += f"Nodes: {result.nodes_expanded}\n"
                info_text += f"Time: {result.time_taken:.4f}s"
                ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
                       verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
            else:
                ax.text(0.5, 0.5, f"{alg_name}\nFAILED", transform=ax.transAxes,
                       ha='center', va='center', fontsize=14, fontweight='bold',
                       bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))
            
            ax.set_title(f"{alg_name}", fontsize=12, fontweight='bold')
            ax.set_xlim(-0.5, self.env.width - 0.5)
            ax.set_ylim(-0.5, self.env.height - 0.5)
            ax.grid(True, alpha=0.3)
        
        # Hide unused subplots
        for i in range(len(algorithms), len(axes)):
            axes[i].set_visible(False)
        plt.suptitle("Algorithm Path Comparison", fontsize=16, fontweight='bold')
        plt.tight_layout()
        return fig

    def _draw_environment_base(self, ax):
        """Draw the base environment on an axis."""
        grid_colors = np.zeros((self.env.height, self.env.width, 3))
        for y in range(self.env.height):
            for x in range(self.env.width):
                terrain = self.env.grid[y][x]
                if terrain == TerrainType.ROAD:
                    color = [1, 1, 1]  # White
                elif terrain == TerrainType.GRASS:
                    color = [0.56, 0.93, 0.56]  # Light green
                elif terrain == TerrainType.WATER:
                    color = [0.53, 0.81, 0.92]  # Sky blue
                elif terrain == TerrainType.MOUNTAIN:
                    color = [0.55, 0.45, 0.33]  # Brown
                elif terrain == TerrainType.BUILDING:
                    color = [0.41, 0.41, 0.41]  # Dark gray
                else:
                    color = [0.5, 0.5, 0.5]  # Gray
                grid_colors[y, x] = color
        ax.imshow(grid_colors, origin='lower')
    
    def create_performance_comparison_chart(self, results: Dict[str, Any]) -> plt.Figure:
        """Create performance comparison charts."""
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        algorithms = list(results.keys())
        success_rates = []
        avg_costs = []
        avg_times = []
        avg_nodes = []
        for alg_name in algorithms:
            result = results[alg_name]
            success_rates.append(1.0 if result.success else 0.0)
            avg_costs.append(result.cost if result.success else 0)
            avg_times.append(result.time_taken)
            avg_nodes.append(result.nodes_expanded)
        
        # Success rates
        axes[0, 0].bar(algorithms, success_rates, color='green', alpha=0.7)
        axes[0, 0].set_title('Success Rates')
        axes[0, 0].set_ylabel('Success Rate')
        axes[0, 0].set_ylim(0, 1.1)
        axes[0, 0].tick_params(axis='x', rotation=45)
        
        # Average costs
        axes[0, 1].bar(algorithms, avg_costs, color='blue', alpha=0.7)
        axes[0, 1].set_title('Average Path Costs')
        axes[0, 1].set_ylabel('Cost')
        axes[0, 1].tick_params(axis='x', rotation=45)
        
        # Average times
        axes[1, 0].bar(algorithms, avg_times, color='red', alpha=0.7)
        axes[1, 0].set_title('Average Execution Times')
        axes[1, 0].set_ylabel('Time (seconds)')
        axes[1, 0].tick_params(axis='x', rotation=45)
        
        # Nodes expanded
        axes[1, 1].bar(algorithms, avg_nodes, color='orange', alpha=0.7)
        axes[1, 1].set_title('Nodes Expanded')
        axes[1, 1].set_ylabel('Number of Nodes')
        axes[1, 1].tick_params(axis='x', rotation=45)
        plt.suptitle('Algorithm Performance Comparison', fontsize=16, fontweight='bold')
        plt.tight_layout()
        return fig
    
    def create_dynamic_replanning_timeline(self, replanning_events: List[Dict]) -> plt.Figure:
        """Create a timeline visualization of replanning events."""
        if not replanning_events:
            fig, ax = plt.subplots(figsize=(12, 6))
            ax.text(0.5, 0.5, 'No Replanning Events Occurred', 
                   ha='center', va='center', fontsize=16, fontweight='bold')
            ax.set_title('Dynamic Replanning Timeline')
            return fig
        fig, ax = plt.subplots(figsize=(12, 8))
        steps = [event['step'] for event in replanning_events]
        positions_x = [event['agent_position'][0] for event in replanning_events]
        positions_y = [event['agent_position'][1] for event in replanning_events]
        
        # Plot replanning events
        ax.scatter(steps, positions_x, c='red', s=100, alpha=0.7, label='Agent X Position')
        ax.scatter(steps, positions_y, c='blue', s=100, alpha=0.7, label='Agent Y Position')
        
        # Connect points
        ax.plot(steps, positions_x, 'red', alpha=0.3, linewidth=2)
        ax.plot(steps, positions_y, 'blue', alpha=0.3, linewidth=2)
        
        # Add event annotations
        for i, event in enumerate(replanning_events):
            ax.annotate(f'Event {i+1}', 
                       (event['step'], event['agent_position'][0]),
                       xytext=(5, 5), textcoords='offset points',
                       fontsize=8, alpha=0.8)
        ax.set_xlabel('Simulation Step')
        ax.set_ylabel('Position')
        ax.set_title('Dynamic Replanning Events Timeline')
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        return fig
    
    def save_visualization(self, fig: plt.Figure, filename: str, dpi: int = 300):
        """Save visualization to file."""
        fig.savefig(filename, dpi=dpi, bbox_inches='tight')
        print(f"Visualization saved to {filename}")

class InteractiveSimulation:
    """Interactive simulation with real-time visualization."""
    def __init__(self, environment: GridEnvironment, agent: DeliveryAgent):
        self.env = environment
        self.agent = agent
        self.visualizer = AgentVisualizer(environment, agent)
        self.fig = None
        self.ax = None
        self.animation = None

    def run_interactive_simulation(self, max_steps: int = 100, 
                                 update_interval: int = 500) -> None:
        """Run interactive simulation with real-time visualization."""
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        def animate(frame):
            self.ax.clear()
            # Run one step of simulation
            if self.agent.state.fuel > 0:
                # Check if we need to plan
                if not self.agent.current_path or self.agent.path_index >= len(self.agent.current_path):
                    self.agent.plan_next_action()
                # Execute next action
                self.agent.execute_next_action()
                # Advance environment time
                self.env.advance_time()
            # Update visualization
            self.visualizer.create_environment_visualization()
            # Add frame info
            self.ax.text(0.02, 0.02, f"Frame: {frame}\nStep: {self.env.current_time}", 
                        transform=self.ax.transAxes, fontsize=10,
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        self.animation = animation.FuncAnimation(
            self.fig, animate, frames=max_steps, 
            interval=update_interval, repeat=False
        )
        plt.title("Interactive Agent Simulation")
        plt.show()
        return self.animation

def create_comprehensive_visualization_report(environment: GridEnvironment, 
                                            agent: DeliveryAgent,
                                            algorithm_results: Dict[str, SearchResult],
                                            replanning_events: List[Dict] = None) -> None:
    """Create a comprehensive visualization report."""
    visualizer = AgentVisualizer(environment, agent)
    print("Creating comprehensive visualization report...")
    # 1. Environment overview
    fig1 = visualizer.create_environment_visualization(
        "Environment Overview with Agent and Tasks", 
        show_path=True, show_dynamic_obstacles=True
    )
    visualizer.save_visualization(fig1, "environment_overview.png")
    plt.close(fig1)
    # 2. Algorithm comparison
    if algorithm_results:
        fig2 = visualizer.create_path_comparison_plot(algorithm_results, 
                                                     agent.state.position, 
                                                     agent.state.position)  # Placeholder goal
        visualizer.save_visualization(fig2, "algorithm_comparison.png")
        plt.close(fig2)
        fig3 = visualizer.create_performance_comparison_chart(algorithm_results)
        visualizer.save_visualization(fig3, "performance_comparison.png")
        plt.close(fig3)
    # 3. Dynamic replanning timeline
    if replanning_events:
        fig4 = visualizer.create_dynamic_replanning_timeline(replanning_events)
        visualizer.save_visualization(fig4, "replanning_timeline.png")
        plt.close(fig4)
    print("Visualization report completed! Check the generated PNG files.")

if __name__ == "__main__":
    # Test visualization tools
    from environment import create_test_environment_dynamic
    from delivery_agent import DeliveryAgent, PlanningStrategy, DeliveryTask
    print("Testing visualization tools...")
    # Create test environment and agent
    env = create_test_environment_dynamic()
    agent = DeliveryAgent(env, Position(0, 0), initial_fuel=50.0)
    # Add test tasks
    tasks = [
        DeliveryTask("test_package", Position(2, 2), Position(12, 12), priority=1)
    ]
    for task in tasks:
        agent.add_delivery_task(task)
    # Create visualizer
    visualizer = AgentVisualizer(env, agent)
    # Test environment visualization
    fig = visualizer.create_environment_visualization("Test Environment")
    visualizer.save_visualization(fig, "test_environment.png")
    plt.close(fig)
    print("Visualization test completed!")
