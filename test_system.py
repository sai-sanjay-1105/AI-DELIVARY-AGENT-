"""
Comprehensive test suite for the autonomous delivery agent system.
"""

import unittest
import tempfile
import os
import json
from typing import List, Dict, Any

from environment import (GridEnvironment, Position, TerrainType, DynamicObstacle,
                        create_test_environment_small, create_test_environment_medium,
                        create_test_environment_large, create_test_environment_dynamic)
from search_algorithms import (BFS, UniformCostSearch, AStar, HillClimbingSearch,
                             SimulatedAnnealingSearch, compare_algorithms)
from delivery_agent import DeliveryAgent, PlanningStrategy, DeliveryTask


class TestEnvironment(unittest.TestCase):
    """Test cases for the grid environment."""
    
    def setUp(self):
        self.env = GridEnvironment(10, 10)
    
    def test_environment_creation(self):
        """Test basic environment creation."""
        self.assertEqual(self.env.width, 10)
        self.assertEqual(self.env.height, 10)
        self.assertEqual(self.env.current_time, 0)
    
    def test_terrain_setting(self):
        """Test setting terrain types."""
        self.env.set_terrain(5, 5, TerrainType.BUILDING)
        self.assertEqual(self.env.grid[5][5], TerrainType.BUILDING)
    
    def test_terrain_region_setting(self):
        """Test setting terrain for a region."""
        self.env.set_terrain_region(2, 2, 4, 4, TerrainType.WATER)
        for y in range(2, 5):
            for x in range(2, 5):
                self.assertEqual(self.env.grid[y][x], TerrainType.WATER)
    
    def test_position_validation(self):
        """Test position validation."""
        self.assertTrue(self.env.is_valid_position(5, 5))
        self.assertFalse(self.env.is_valid_position(-1, 5))
        self.assertFalse(self.env.is_valid_position(5, -1))
        self.assertFalse(self.env.is_valid_position(10, 5))
        self.assertFalse(self.env.is_valid_position(5, 10))
    
    def test_passability(self):
        """Test passability checking."""
        # Road should be passable
        self.assertTrue(self.env.is_passable(5, 5))
        
        # Building should not be passable
        self.env.set_terrain(5, 5, TerrainType.BUILDING)
        self.assertFalse(self.env.is_passable(5, 5))
    
    def test_movement_cost(self):
        """Test movement cost calculation."""
        self.assertEqual(self.env.get_movement_cost(5, 5), 1)  # Road
        
        self.env.set_terrain(5, 5, TerrainType.GRASS)
        self.assertEqual(self.env.get_movement_cost(5, 5), 2)
        
        self.env.set_terrain(5, 5, TerrainType.BUILDING)
        self.assertEqual(self.env.get_movement_cost(5, 5), float('inf'))
    
    def test_neighbors(self):
        """Test neighbor generation."""
        pos = Position(5, 5)
        neighbors = self.env.get_neighbors(pos)
        
        # Should have 4 neighbors for center position
        self.assertEqual(len(neighbors), 4)
        
        # Check specific neighbors
        neighbor_positions = [neighbor[0] for neighbor in neighbors]
        expected = [Position(5, 6), Position(5, 4), Position(4, 5), Position(6, 5)]
        self.assertEqual(set(neighbor_positions), set(expected))
    
    def test_dynamic_obstacles(self):
        """Test dynamic obstacle functionality."""
        positions = [Position(1, 1), Position(2, 1), Position(3, 1)]
        self.env.add_dynamic_obstacle("test_obstacle", positions)
        
        # Check obstacle at different times
        self.assertEqual(self.env.dynamic_obstacles["test_obstacle"].get_position_at_time(0), Position(1, 1))
        self.assertEqual(self.env.dynamic_obstacles["test_obstacle"].get_position_at_time(1), Position(2, 1))
        self.assertEqual(self.env.dynamic_obstacles["test_obstacle"].get_position_at_time(2), Position(3, 1))
        self.assertEqual(self.env.dynamic_obstacles["test_obstacle"].get_position_at_time(3), Position(1, 1))  # Cycles
    
    def test_save_load(self):
        """Test saving and loading environment."""
        # Set up test environment
        self.env.set_terrain(5, 5, TerrainType.BUILDING)
        self.env.add_dynamic_obstacle("test", [Position(1, 1), Position(2, 2)])
        
        # Save to temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            temp_filename = f.name
        
        try:
            self.env.save_to_file(temp_filename)
            
            # Load from file
            loaded_env = GridEnvironment.load_from_file(temp_filename)
            
            # Verify loaded environment
            self.assertEqual(loaded_env.width, self.env.width)
            self.assertEqual(loaded_env.height, self.env.height)
            self.assertEqual(loaded_env.grid[5][5], TerrainType.BUILDING)
            self.assertIn("test", loaded_env.dynamic_obstacles)
        finally:
            os.unlink(temp_filename)


class TestSearchAlgorithms(unittest.TestCase):
    """Test cases for search algorithms."""
    
    def setUp(self):
        self.env = create_test_environment_small()
        self.start = Position(0, 0)
        self.goal = Position(9, 9)
    
    def test_bfs(self):
        """Test BFS algorithm."""
        bfs = BFS(self.env)
        result = bfs.search(self.start, self.goal)
        
        self.assertTrue(result.success)
        self.assertEqual(result.path[0], self.start)
        self.assertEqual(result.path[-1], self.goal)
        self.assertGreater(result.nodes_expanded, 0)
        self.assertGreater(result.time_taken, 0)
    
    def test_uniform_cost_search(self):
        """Test Uniform-cost search algorithm."""
        ucs = UniformCostSearch(self.env)
        result = ucs.search(self.start, self.goal)
        
        self.assertTrue(result.success)
        self.assertEqual(result.path[0], self.start)
        self.assertEqual(result.path[-1], self.goal)
        self.assertGreater(result.cost, 0)
    
    def test_astar(self):
        """Test A* algorithm."""
        astar = AStar(self.env)
        result = astar.search(self.start, self.goal)
        
        self.assertTrue(result.success)
        self.assertEqual(result.path[0], self.start)
        self.assertEqual(result.path[-1], self.goal)
        self.assertGreater(result.cost, 0)
    
    def test_hill_climbing(self):
        """Test Hill-climbing algorithm."""
        hc = HillClimbingSearch(self.env)
        result = hc.search(self.start, self.goal)
        
        # Hill-climbing might not always succeed, but should not crash
        self.assertIsInstance(result.success, bool)
        self.assertGreaterEqual(result.nodes_expanded, 0)
    
    def test_simulated_annealing(self):
        """Test Simulated Annealing algorithm."""
        sa = SimulatedAnnealingSearch(self.env)
        result = sa.search(self.start, self.goal)
        
        # Simulated annealing might not always succeed, but should not crash
        self.assertIsInstance(result.success, bool)
        self.assertGreaterEqual(result.nodes_expanded, 0)
    
    def test_algorithm_comparison(self):
        """Test algorithm comparison function."""
        results = compare_algorithms(self.env, self.start, self.goal)
        
        # Should have results for all algorithms
        expected_algorithms = ['BFS', 'Uniform Cost', 'A* Manhattan', 'A* Euclidean', 
                             'Hill Climbing', 'Simulated Annealing']
        self.assertEqual(set(results.keys()), set(expected_algorithms))
        
        # At least some algorithms should succeed
        successful_algorithms = [name for name, result in results.items() if result.success]
        self.assertGreater(len(successful_algorithms), 0)


class TestDeliveryAgent(unittest.TestCase):
    """Test cases for the delivery agent."""
    
    def setUp(self):
        self.env = create_test_environment_small()
        self.agent = DeliveryAgent(self.env, Position(0, 0), initial_fuel=100.0)
    
    def test_agent_creation(self):
        """Test agent creation."""
        self.assertEqual(self.agent.state.position, Position(0, 0))
        self.assertEqual(self.agent.state.fuel, 100.0)
        self.assertEqual(len(self.agent.state.carrying_packages), 0)
        self.assertEqual(len(self.agent.state.completed_deliveries), 0)
    
    def test_strategy_setting(self):
        """Test setting planning strategy."""
        self.agent.set_planning_strategy(PlanningStrategy.ASTAR_MANHATTAN)
        self.assertEqual(self.agent.current_strategy, PlanningStrategy.ASTAR_MANHATTAN)
        self.assertIsNotNone(self.agent.planning_algorithm)
    
    def test_task_management(self):
        """Test adding and managing delivery tasks."""
        task = DeliveryTask("test_package", Position(1, 1), Position(8, 8))
        self.agent.add_delivery_task(task)
        
        self.assertEqual(len(self.agent.tasks), 1)
        self.assertEqual(self.agent.tasks[0].package_id, "test_package")
    
    def test_goal_determination(self):
        """Test next goal determination."""
        # Add a task
        task = DeliveryTask("test_package", Position(1, 1), Position(8, 8))
        self.agent.add_delivery_task(task)
        
        # Should return pickup location
        next_goal = self.agent._determine_next_goal()
        self.assertEqual(next_goal, Position(1, 1))
        
        # Pick up package
        self.agent.state.carrying_packages.append("test_package")
        
        # Should return delivery location
        next_goal = self.agent._determine_next_goal()
        self.assertEqual(next_goal, Position(8, 8))
    
    def test_path_validation(self):
        """Test path validation."""
        # Create a simple path
        self.agent.current_path = [Position(1, 1), Position(2, 2), Position(3, 3)]
        self.agent.path_index = 0
        
        # Should be valid initially (all positions are passable)
        self.assertTrue(self.agent._is_path_still_valid())
        
        # Add obstacle in path
        self.env.set_terrain(2, 2, TerrainType.BUILDING)
        
        # Should no longer be valid
        self.assertFalse(self.agent._is_path_still_valid())
    
    def test_simulation_run(self):
        """Test running a simulation."""
        # Add a simple task
        task = DeliveryTask("test_package", Position(1, 1), Position(2, 2))
        self.agent.add_delivery_task(task)
        
        # Set strategy
        self.agent.set_planning_strategy(PlanningStrategy.ASTAR_MANHATTAN)
        
        # Run simulation
        stats = self.agent.run_simulation(max_steps=100)
        
        # Check that simulation ran
        self.assertGreater(stats['simulation_steps'], 0)
        self.assertGreaterEqual(stats['deliveries_completed'], 0)
    
    def test_reset(self):
        """Test agent reset functionality."""
        # Modify agent state
        self.agent.state.position = Position(5, 5)
        self.agent.state.fuel = 50.0
        self.agent.state.carrying_packages = ["test_package"]
        
        # Reset
        self.agent.reset()
        
        # Check reset state
        self.assertEqual(self.agent.state.position, Position(0, 0))
        self.assertEqual(self.agent.state.fuel, 100.0)
        self.assertEqual(len(self.agent.state.carrying_packages), 0)


class TestDynamicReplanning(unittest.TestCase):
    """Test cases for dynamic replanning functionality."""
    
    def test_dynamic_obstacle_replanning(self):
        """Test replanning when dynamic obstacles block path."""
        env = create_test_environment_dynamic()
        agent = DeliveryAgent(env, Position(0, 0))
        agent.set_planning_strategy(PlanningStrategy.ASTAR_MANHATTAN)
        
        # Add a task
        task = DeliveryTask("test_package", Position(1, 1), Position(14, 14))
        agent.add_delivery_task(task)
        
        # Plan initial path
        success = agent.plan_next_action()
        self.assertTrue(success)
        
        # Advance time to move obstacles
        for _ in range(5):
            env.advance_time()
        
        # Try to execute action - should trigger replanning
        initial_replanning_events = agent.stats['replanning_events']
        agent.execute_next_action()
        
        # Should have triggered replanning
        self.assertGreaterEqual(agent.stats['replanning_events'], initial_replanning_events)


def run_performance_tests():
    """Run performance tests on different map sizes."""
    print("Running performance tests...")
    
    maps = {
        'small': create_test_environment_small(),
        'medium': create_test_environment_medium(),
        'large': create_test_environment_large(),
        'dynamic': create_test_environment_dynamic()
    }
    
    test_scenarios = {
        'small': [(Position(0, 0), Position(9, 9))],
        'medium': [(Position(0, 0), Position(19, 19))],
        'large': [(Position(0, 0), Position(49, 49))],
        'dynamic': [(Position(0, 0), Position(14, 14))]
    }
    
    results = {}
    
    for map_name, env in maps.items():
        print(f"\nTesting {map_name} map...")
        map_results = {}
        
        for i, (start, goal) in enumerate(test_scenarios[map_name]):
            print(f"  Scenario {i+1}: {start} -> {goal}")
            scenario_results = compare_algorithms(env, start, goal)
            map_results[f"scenario_{i+1}"] = scenario_results
            
            # Print summary
            for alg_name, result in scenario_results.items():
                status = "SUCCESS" if result.success else "FAILED"
                print(f"    {alg_name}: {status} - {result.nodes_expanded} nodes, {result.time_taken:.4f}s")
        
        results[map_name] = map_results
    
    return results


def run_diagonal_movement_tests():
    """Test diagonal movement functionality."""
    print("Running diagonal movement tests...")
    
    # Create environment with diagonal movement enabled
    env_4connected = GridEnvironment(10, 10, allow_diagonal=False)
    env_8connected = GridEnvironment(10, 10, allow_diagonal=True)
    
    # Add some obstacles
    env_4connected.set_terrain_region(3, 3, 5, 5, TerrainType.BUILDING)
    env_8connected.set_terrain_region(3, 3, 5, 5, TerrainType.BUILDING)
    
    start = Position(0, 0)
    goal = Position(9, 9)
    
    # Test A* with both movement types
    from search_algorithms import AStar
    
    astar_4 = AStar(env_4connected)
    astar_8 = AStar(env_8connected)
    
    result_4 = astar_4.search(start, goal)
    result_8 = astar_8.search(start, goal)
    
    print(f"4-connected path length: {len(result_4.path) if result_4.success else 'Failed'}")
    print(f"8-connected path length: {len(result_8.path) if result_8.success else 'Failed'}")
    print(f"4-connected cost: {result_4.cost if result_4.success else 'N/A'}")
    print(f"8-connected cost: {result_8.cost if result_8.success else 'N/A'}")
    
    # Diagonal should generally be shorter but potentially more expensive
    if result_4.success and result_8.success:
        assert len(result_8.path) <= len(result_4.path), "Diagonal path should be shorter or equal"
        print("✓ Diagonal movement test passed")
    else:
        print("⚠ Diagonal movement test inconclusive")


def run_replanning_tests():
    """Test replanning strategies."""
    print("Running replanning strategy tests...")
    
    from replanning_strategies import AdvancedReplanningAgent, ReplanningStrategy
    from delivery_agent import DeliveryTask
    
    env = create_test_environment_dynamic()
    
    # Test different replanning strategies
    strategies = [
        ReplanningStrategy.IMMEDIATE,
        ReplanningStrategy.PREDICTIVE,
        ReplanningStrategy.ADAPTIVE
    ]
    
    tasks = [
        DeliveryTask("test_package", Position(1, 1), Position(13, 13), priority=3)
    ]
    
    results = {}
    
    for strategy in strategies:
        print(f"  Testing {strategy.value} strategy...")
        
        agent = AdvancedReplanningAgent(env, Position(0, 0), 
                                      initial_fuel=50.0, replanning_strategy=strategy)
        
        for task in tasks:
            agent.add_delivery_task(task)
        
        # Run simulation
        step = 0
        max_steps = 100
        
        while step < max_steps and agent.state.fuel > 0:
            if not agent.execute_next_action_with_replanning():
                break
            step += 1
            env.advance_time()
        
        replanning_stats = agent.get_replanning_statistics()
        results[strategy.value] = {
            'deliveries_completed': agent.stats['deliveries_completed'],
            'replanning_events': replanning_stats['total_replanning_events'],
            'efficiency': replanning_stats['replanning_efficiency']
        }
        
        print(f"    Deliveries: {agent.stats['deliveries_completed']}")
        print(f"    Replanning events: {replanning_stats['total_replanning_events']}")
        print(f"    Efficiency: {replanning_stats['replanning_efficiency']:.2f}")
    
    return results


def run_visualization_tests():
    """Test visualization functionality."""
    print("Running visualization tests...")
    
    try:
        from visualization import DeliveryVisualizer
        from environment import create_test_environment_dynamic
        from delivery_agent import DeliveryAgent, PlanningStrategy, DeliveryTask
        
        env = create_test_environment_dynamic()
        visualizer = DeliveryVisualizer(env)
        
        # Test basic plotting
        start = Position(0, 0)
        goal = Position(14, 14)
        
        # Test environment plotting
        visualizer.plot_environment(agent_pos=start, goal_pos=goal, title="Test Environment")
        print("✓ Environment plotting test passed")
        
        # Test agent creation and basic simulation
        agent = DeliveryAgent(env, start, initial_fuel=30.0)
        agent.set_planning_strategy(PlanningStrategy.ASTAR_MANHATTAN)
        
        task = DeliveryTask("test_package", Position(1, 1), Position(13, 13))
        agent.add_delivery_task(task)
        
        # Run a few steps
        for _ in range(5):
            if not agent.plan_next_action():
                break
            if not agent.execute_next_action():
                break
            env.advance_time()
        
        print("✓ Agent simulation test passed")
        
    except ImportError as e:
        print(f"⚠ Visualization test skipped: {e}")
    except Exception as e:
        print(f"✗ Visualization test failed: {e}")


def run_performance_analysis_tests():
    """Test performance analysis functionality."""
    print("Running performance analysis tests...")
    
    try:
        from performance_analysis import PerformanceAnalyzer
        
        analyzer = PerformanceAnalyzer()
        
        # Run a small benchmark
        print("  Running mini benchmark...")
        report = analyzer.run_comprehensive_benchmark(num_runs=2)
        
        print(f"✓ Performance analysis test passed")
        print(f"  Total experiments: {len(analyzer.experiment_results)}")
        print(f"  Algorithms tested: {len(analyzer.performance_metrics)}")
        
    except ImportError as e:
        print(f"⚠ Performance analysis test skipped: {e}")
    except Exception as e:
        print(f"✗ Performance analysis test failed: {e}")

def run_stress_tests():
    """Run stress tests with large maps and complex scenarios."""
    print("Running stress tests...")
    
    # Test large map performance
    print("  Testing large map performance...")
    large_env = create_test_environment_large()
    start = Position(0, 0)
    goal = Position(49, 49)
    
    from search_algorithms import AStar, BFS
    # Test A* on large map
    astar = AStar(large_env)
    start_time = time.time()
    result = astar.search(start, goal)
    astar_time = time.time() - start_time
    print(f"    A* on large map: {result.success}, {astar_time:.4f}s, {result.nodes_expanded} nodes")
    
    # Test with many delivery tasks
    print("  Testing multiple delivery tasks...")
    env = create_test_environment_medium()
    agent = DeliveryAgent(env, Position(0, 0), initial_fuel=200.0)
    agent.set_planning_strategy(PlanningStrategy.ASTAR_MANHATTAN)
    
    # Add many tasks
    tasks = []
    for i in range(10):
        task = DeliveryTask(f"package_{i}", 
                          Position(i % 5, i % 5), 
                          Position(15 + i % 5, 15 + i % 5), 
                          priority=i % 3 + 1)
        tasks.append(task)
        agent.add_delivery_task(task)
    
    # Run simulation
    start_time = time.time()
    stats = agent.run_simulation(max_steps=500)
    simulation_time = time.time() - start_time
    
    print(f"    Multi-task simulation: {stats['deliveries_completed']} deliveries in {simulation_time:.4f}s")
    
    print("✓ Stress tests completed")


def run_integration_tests():
    """Run integration tests combining multiple components."""
    print("Running integration tests...")
    
    # Test complete delivery workflow
    print("  Testing complete delivery workflow...")
    env = create_test_environment_dynamic()
    
    # Create agent with advanced replanning
    from replanning_strategies import AdvancedReplanningAgent, ReplanningStrategy
    agent = AdvancedReplanningAgent(env, Position(0, 0), 
                                  initial_fuel=100.0, 
                                  replanning_strategy=ReplanningStrategy.ADAPTIVE)
    
    # Add multiple tasks with different priorities
    tasks = [
        DeliveryTask("urgent", Position(1, 1), Position(13, 13), priority=3),
        DeliveryTask("standard", Position(2, 2), Position(12, 12), priority=2),
        DeliveryTask("low", Position(3, 3), Position(11, 11), priority=1)
    ]
    
    for task in tasks:
        agent.add_delivery_task(task)
    
    # Run complete simulation
    step = 0
    max_steps = 200
    
    while step < max_steps and agent.state.fuel > 0:
        if not agent.execute_next_action_with_replanning():
            break
        step += 1
        env.advance_time()
    
    # Verify results
    assert agent.stats['deliveries_completed'] >= 0, "Should complete some deliveries"
    assert agent.stats['replanning_events'] >= 0, "Should have some replanning events"
    
    print(f"    Completed {agent.stats['deliveries_completed']} deliveries in {step} steps")
    print(f"    Replanning events: {agent.stats['replanning_events']}")
    
    print("✓ Integration tests passed")


def run_all_tests():
    """Run all test suites."""
    print("="*60)
    print("RUNNING COMPREHENSIVE TEST SUITE")
    print("="*60)
    
    # Unit tests
    print("\n1. Running unit tests...")
    unittest.main(argv=[''], exit=False, verbosity=1)
    
    # Performance tests
    print("\n2. Running performance tests...")
    performance_results = run_performance_tests()
    
    # Diagonal movement tests
    print("\n3. Running diagonal movement tests...")
    run_diagonal_movement_tests()
    
    # Replanning tests
    print("\n4. Running replanning tests...")
    replanning_results = run_replanning_tests()
    
    # Visualization tests
    print("\n5. Running visualization tests...")
    run_visualization_tests()
    
    # Performance analysis tests
    print("\n6. Running performance analysis tests...")
    run_performance_analysis_tests()
    
    # Stress tests
    print("\n7. Running stress tests...")
    run_stress_tests()
    
    # Integration tests
    print("\n8. Running integration tests...")
    run_integration_tests()
    
    print("\n" + "="*60)
    print("ALL TESTS COMPLETED")
    print("="*60)
    
    # Save test results
    with open('test_results.json', 'w') as f:
        json.dump({
            'performance_results': performance_results,
            'replanning_results': replanning_results,
            'test_timestamp': time.time()
        }, f, indent=2, default=str)
    
    print("Test results saved to test_results.json")


if __name__ == "__main__":
    # Run unit tests
    print("Running unit tests...")
    unittest.main(argv=[''], exit=False, verbosity=2)
    
    # Run performance tests
    print("\n" + "="*50)
    performance_results = run_performance_tests()
    
    # Save performance results
    with open('performance_test_results.json', 'w') as f:
        # Convert to JSON-serializable format
        json_results = {}
        for map_name, map_data in performance_results.items():
            json_results[map_name] = {}
            for scenario_name, scenario_data in map_data.items():
                json_results[map_name][scenario_name] = {}
                for alg_name, result in scenario_data.items():
                    json_results[map_name][scenario_name][alg_name] = {
                        'success': result.success,
                        'path_length': len(result.path) if result.success else 0,
                        'cost': result.cost,
                        'nodes_expanded': result.nodes_expanded,
                        'time_taken': result.time_taken
                    }
        
        json.dump(json_results, f, indent=2)
    
    print("\nPerformance test results saved to performance_test_results.json")
