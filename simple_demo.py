from environment import create_test_environment_small, Position
from search_algorithms import compare_algorithms
from delivery_agent import DeliveryAgent, PlanningStrategy, DeliveryTask

def simple_algorithm_comparison():
    """Simple algorithm comparison demonstration."""
    print("=" * 60)
    print("SIMPLE ALGORITHM COMPARISON")
    print("=" * 60)
    env = create_test_environment_small()
    start = Position(0, 0)
    goal = Position(9, 9)
    print(f"Comparing algorithms from {start} to {goal}")
    print("Environment: 10x10 grid with buildings and terrain variety")
    
    # Show environment
    print("\nEnvironment (A=Start, G=Goal, #=Building, g=Grass):")
    symbols = {1: '.', 2: 'g', 3: '~', 4: '^', 5: '#'}
    for y in range(env.height):
        row = ""
        for x in range(env.width):
            if Position(x, y) == start:
                row += "A"
            elif Position(x, y) == goal:
                row += "G"
            else:
                terrain = env.grid[y][x]
                row += symbols.get(terrain.value, '.')
        print(row)
    
    # Compare algorithms
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

def simple_delivery_simulation():
    """Simple delivery simulation demonstration."""
    print("\n" + "=" * 60)
    print("SIMPLE DELIVERY SIMULATION")
    print("=" * 60)
    env = create_test_environment_small()
    agent = DeliveryAgent(env, Position(0, 0), initial_fuel=30.0)
    agent.set_planning_strategy(PlanningStrategy.ASTAR_MANHATTAN)
    # Add simple delivery task
    task = DeliveryTask("test_package", Position(1, 1), Position(8, 8), priority=1)
    agent.add_delivery_task(task)
    print(f"Delivery Task: {task.package_id}")
    print(f"  Pickup: {task.pickup_location}")
    print(f"  Delivery: {task.delivery_location}")
    print(f"  Strategy: {agent.current_strategy.value}")
    print("\nRunning simulation...")
    
    # Run simulation
    stats = agent.run_simulation(max_steps=30)
    print(f"\nSimulation Results:")
    print(f"  Deliveries completed: {stats['deliveries_completed']}")
    print(f"  Steps taken: {stats['simulation_steps']}")
    print(f"  Fuel consumed: {stats['total_fuel_consumed']:.2f}")
    print(f"  Planning time: {stats['total_planning_time']:.4f}s")
    print(f"  Replanning events: {stats['replanning_events']}")

def strategy_comparison():
    """Compare different planning strategies."""
    print("\n" + "=" * 60)
    print("STRATEGY COMPARISON")
    print("=" * 60)
    strategies = [
        PlanningStrategy.BFS,
        PlanningStrategy.ASTAR_MANHATTAN,
        PlanningStrategy.UNIFORM_COST
    ]
    env = create_test_environment_small()
    task = DeliveryTask("test_package", Position(1, 1), Position(8, 8))
    print("Comparing strategies on same delivery task:")
    print(f"  Package: {task.pickup_location} -> {task.delivery_location}")
    results = {}
    
    for strategy in strategies:
        print(f"\nTesting {strategy.value}...")
        agent = DeliveryAgent(env, Position(0, 0), initial_fuel=25.0)
        agent.set_planning_strategy(strategy)
        agent.add_delivery_task(task)
        # Run simulation
        stats = agent.run_simulation(max_steps=25)
        results[strategy.value] = stats
        print(f"  Result: {stats['deliveries_completed']} deliveries in {stats['simulation_steps']} steps")
        print(f"  Fuel used: {stats['total_fuel_consumed']:.2f}")
    
    # Summary table
    print(f"\nStrategy Comparison Summary:")
    print("-" * 60)
    print(f"{'Strategy':<20} {'Deliveries':<12} {'Steps':<8} {'Fuel Used':<12}")
    print("-" * 60)
    for strategy_name, stats in results.items():
        print(f"{strategy_name:<20} {stats['deliveries_completed']:<12} {stats['simulation_steps']:<8} "
              f"{stats['total_fuel_consumed']:<12.2f}")

def main():
    """Run simple demonstrations."""
    print("AUTONOMOUS DELIVERY AGENT SYSTEM - SIMPLE DEMO")
    print("=" * 60)
    try:
        simple_algorithm_comparison()
        simple_delivery_simulation()
        strategy_comparison()
        print("\n" + "=" * 60)
        print("DEMONSTRATION COMPLETE")
        print("=" * 60)
        print("Key Features Demonstrated:")
        print("✓ Multiple search algorithms (BFS, A*, Uniform Cost)")
        print("✓ Delivery agent with task management")
        print("✓ Different planning strategies")
        print("✓ Performance comparison")
        print("\nTo run full experiments:")
        print("  python cli.py experiment")
        print("  python cli.py analyze")
    except Exception as e:
        print(f"Error during demonstration: {e}")
        print("This is expected for some edge cases in the current implementation.")

if __name__ == "__main__":
    main()
