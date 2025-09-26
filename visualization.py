import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from typing import List, Dict, Any, Optional, Tuple
import time
from environment import GridEnvironment, Position, TerrainType
from delivery_agent import DeliveryAgent, DeliveryTask
from search_algorithms import SearchResult

class DeliveryVisualizer:
    """Advanced visualizer for delivery agent demonstrations."""
    def __init__(self, environment: GridEnvironment, figsize: Tuple[int, int] = (12, 10)):
        self.env = environment
        self.fig, self.ax = plt.subplots(figsize=figsize)
        self.setup_colors()
        
    def setup_colors(self):
        """Setup color scheme for different terrain types."""
        self.terrain_colors = {
            TerrainType.ROAD: '#E8E8E8',      # Light gray
            TerrainType.GRASS: '#90EE90',      # Light green
            TerrainType.WATER: '#87CEEB',      # Sky blue
            TerrainType.MOUNTAIN: '#8B4513',   # Saddle brown
            TerrainType.BUILDING: '#2F2F2F'    # Dark gray
        }
        self.terrain_names = {
            TerrainType.ROAD: 'Road',
            TerrainType.GRASS: 'Grass',
            TerrainType.WATER: 'Water',
            TerrainType.MOUNTAIN: 'Mountain',
            TerrainType.BUILDING: 'Building'
        }
    def plot_environment(self, agent_pos: Optional[Position] = None, 
                        goal_pos: Optional[Position] = None,
                        path: Optional[List[Position]] = None,
                        dynamic_obstacles: Optional[List[Position]] = None,
                        title: str = "Environment"):
        """Plot the environment with optional agent, goal, path, and obstacles."""
        self.ax.clear()
        # Create grid visualization
        grid_display = np.zeros((self.env.height, self.env.width))
        for y in range(self.env.height):
            for x in range(self.env.width):
                terrain = self.env.grid[y][x]
                grid_display[y][x] = list(TerrainType).index(terrain)
        
        # Plot terrain
        im = self.ax.imshow(grid_display, cmap='terrain', aspect='equal')
        # Add terrain legend
        self._add_terrain_legend()
        # Plot path if provided
        if path:
            path_x = [pos.x for pos in path]
            path_y = [pos.y for pos in path]
            self.ax.plot(path_x, path_y, 'r-', linewidth=3, alpha=0.7, label='Planned Path')
        # Plot agent position
        if agent_pos:
            self.ax.plot(agent_pos.x, agent_pos.y, 'bo', markersize=15, 
                        markeredgecolor='black', markeredgewidth=2, label='Agent')
        # Plot goal position
        if goal_pos:
            self.ax.plot(goal_pos.x, goal_pos.y, 'go', markersize=15, 
                        markeredgecolor='black', markeredgewidth=2, label='Goal')
        # Plot dynamic obstacles
        if dynamic_obstacles:
            for obs_pos in dynamic_obstacles:
                self.ax.plot(obs_pos.x, obs_pos.y, 'ro', markersize=12, 
                           markeredgecolor='black', markeredgewidth=2, label='Moving Obstacle')
        # Customize plot
        self.ax.set_xlim(-0.5, self.env.width - 0.5)
        self.ax.set_ylim(-0.5, self.env.height - 0.5)
        self.ax.set_xticks(range(self.env.width))
        self.ax.set_yticks(range(self.env.height))
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title(title, fontsize=16, fontweight='bold')
        self.ax.legend(loc='upper right')
        # Invert y-axis to match grid coordinates
        self.ax.invert_yaxis()
        return im
    
    def _add_terrain_legend(self):
        """Add terrain type legend to the plot."""
        legend_elements = []
        for terrain_type, color in self.terrain_colors.items():
            legend_elements.append(plt.Rectangle((0, 0), 1, 1, 
                                               facecolor=color, 
                                               label=self.terrain_names[terrain_type]))
        # Add legend outside the plot
        self.ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1, 0.5))

    def create_animation(self, agent: DeliveryAgent, max_steps: int = 100, 
                        interval: int = 500) -> animation.FuncAnimation:
        """Create an animated visualization of agent movement."""
        def animate(frame):
            if frame < max_steps and agent.state.fuel > 0:
                # Execute one step
                if not agent.current_path or agent.path_index >= len(agent.current_path):
                    if not agent.plan_next_action():
                        return
                if not agent.execute_next_action():
                    return
                # Advance environment time
                self.env.advance_time()
            # Get current state
            agent_pos = agent.state.position
            dynamic_obstacles = list(self.env.get_dynamic_obstacles_at_time(self.env.current_time))
            # Plot current state
            self.plot_environment(
                agent_pos=agent_pos,
                dynamic_obstacles=dynamic_obstacles,
                path=agent.current_path[agent.path_index:] if agent.current_path else None,
                title=f"Delivery Agent - Step {frame} (Fuel: {agent.state.fuel:.1f})"
            )
            return []
        return animation.FuncAnimation(self.fig, animate, frames=max_steps, 
                                     interval=interval, repeat=False, blit=False)
    
    def plot_algorithm_comparison(self, results: Dict[str, SearchResult], 
                                start: Position, goal: Position):
        """Plot comparison of different algorithms."""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('Algorithm Comparison Results', fontsize=16, fontweight='bold')
        algorithms = list(results.keys())
        for i, (alg_name, result) in enumerate(results.items()):
            row = i // 3
            col = i % 3
            ax = axes[row, col]
            # Plot environment
            self._plot_environment_on_axis(ax, start, goal, result.path if result.success else [])
            # Add algorithm info
            status = "SUCCESS" if result.success else "FAILED"
            cost = f"{result.cost:.2f}" if result.success else "N/A"
            nodes = result.nodes_expanded
            time_taken = f"{result.time_taken:.4f}s"
            
            ax.set_title(f"{alg_name}\n{status} - Cost: {cost}, Nodes: {nodes}, Time: {time_taken}")
        
        # Hide unused subplots
        for i in range(len(algorithms), 6):
            row = i // 3
            col = i % 3
            axes[row, col].set_visible(False)
        plt.tight_layout()
        return fig
    def _plot_environment_on_axis(self, ax, start: Position, goal: Position, path: List[Position]):
        """Plot environment on a specific axis."""
        # Create grid visualization
        grid_display = np.zeros((self.env.height, self.env.width))
        for y in range(self.env.height):
            for x in range(self.env.width):
                terrain = self.env.grid[y][x]
                grid_display[y][x] = list(TerrainType).index(terrain)
        
        # Plot terrain
        ax.imshow(grid_display, cmap='terrain', aspect='equal')
        # Plot path
        if path:
            path_x = [pos.x for pos in path]
            path_y = [pos.y for pos in path]
            ax.plot(path_x, path_y, 'r-', linewidth=2, alpha=0.7)
        # Plot start and goal
        ax.plot(start.x, start.y, 'bo', markersize=10, markeredgecolor='black', markeredgewidth=2)
        ax.plot(goal.x, goal.y, 'go', markersize=10, markeredgecolor='black', markeredgewidth=2)
        # Customize
        ax.set_xlim(-0.5, self.env.width - 0.5)
        ax.set_ylim(-0.5, self.env.height - 0.5)
        ax.grid(True, alpha=0.3)
        ax.invert_yaxis()
    
    def plot_performance_metrics(self, results: Dict[str, Dict[str, Any]]):
        """Plot performance metrics comparison."""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Performance Metrics Analysis', fontsize=16, fontweight='bold')
        algorithms = list(results.keys())
        
        # Extract metrics
        success_rates = [1 if results[alg]['success'] else 0 for alg in algorithms]
        costs = [results[alg]['cost'] if results[alg]['success'] else 0 for alg in algorithms]
        nodes_expanded = [results[alg]['nodes_expanded'] for alg in algorithms]
        time_taken = [results[alg]['time_taken'] for alg in algorithms]
        
        # Success rates
        axes[0, 0].bar(algorithms, success_rates, color=['green' if s else 'red' for s in success_rates])
        axes[0, 0].set_title('Success Rate')
        axes[0, 0].set_ylabel('Success (1) / Failure (0)')
        axes[0, 0].tick_params(axis='x', rotation=45)
        
        # Path costs
        axes[0, 1].bar(algorithms, costs, color='skyblue')
        axes[0, 1].set_title('Path Cost')
        axes[0, 1].set_ylabel('Cost')
        axes[0, 1].tick_params(axis='x', rotation=45)
        
        # Nodes expanded
        axes[1, 0].bar(algorithms, nodes_expanded, color='orange')
        axes[1, 0].set_title('Nodes Expanded')
        axes[1, 0].set_ylabel('Number of Nodes')
        axes[1, 0].tick_params(axis='x', rotation=45)
        
        # Time taken
        axes[1, 1].bar(algorithms, time_taken, color='purple')
        axes[1, 1].set_title('Planning Time')
        axes[1, 1].set_ylabel('Time (seconds)')
        axes[1, 1].tick_params(axis='x', rotation=45)
        plt.tight_layout()
        return fig
    
    def save_animation(self, agent: DeliveryAgent, filename: str, 
                      max_steps: int = 100, interval: int = 500):
        """Save animation as GIF or MP4."""
        anim = self.create_animation(agent, max_steps, interval)
        if filename.endswith('.gif'):
            anim.save(filename, writer='pillow', fps=1000//interval)
        elif filename.endswith('.mp4'):
            anim.save(filename, writer='ffmpeg', fps=1000//interval)
        else:
            raise ValueError("Filename must end with .gif or .mp4")
        print(f"Animation saved to {filename}")

def create_demo_visualization():
    """Create a comprehensive demonstration visualization."""
    from environment import create_test_environment_dynamic
    from delivery_agent import DeliveryAgent, PlanningStrategy, DeliveryTask
    from search_algorithms import compare_algorithms
    
    # Create environment
    env = create_test_environment_dynamic()
    
    # Create visualizer
    visualizer = DeliveryVisualizer(env)
    
    # Test different algorithms
    start = Position(0, 0)
    goal = Position(14, 14)
    print("Running algorithm comparison...")
    results = compare_algorithms(env, start, goal)
    
    # Plot comparison
    fig = visualizer.plot_algorithm_comparison(results, start, goal)
    plt.savefig('algorithm_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    # Plot performance metrics
    metrics = {name: {
        'success': result.success,
        'cost': result.cost,
        'nodes_expanded': result.nodes_expanded,
        'time_taken': result.time_taken
    } for name, result in results.items()}
    fig2 = visualizer.plot_performance_metrics(metrics)
    plt.savefig('performance_metrics.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    # Create delivery simulation animation
    agent = DeliveryAgent(env, start, initial_fuel=50.0)
    agent.set_planning_strategy(PlanningStrategy.ASTAR_MANHATTAN)
    
    # Add delivery tasks
    tasks = [
        DeliveryTask("package_1", Position(1, 1), Position(13, 13), priority=3),
        DeliveryTask("package_2", Position(2, 2), Position(12, 12), priority=2),
    ]
    for task in tasks:
        agent.add_delivery_task(task)
    print("Creating delivery simulation animation...")
    visualizer.save_animation(agent, 'delivery_simulation.gif', max_steps=50, interval=800)
    print("Visualization complete!")

if __name__ == "__main__":
    create_demo_visualization()
