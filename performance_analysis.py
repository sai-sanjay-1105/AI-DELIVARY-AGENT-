import time
import json
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Any, Tuple, Optional
from dataclasses import dataclass, asdict
from collections import defaultdict
import statistics
from environment import GridEnvironment, Position, create_test_environment_small, create_test_environment_medium, create_test_environment_large, create_test_environment_dynamic
from delivery_agent import DeliveryAgent, PlanningStrategy, DeliveryTask
from search_algorithms import compare_algorithms, SearchResult
from replanning_strategies import AdvancedReplanningAgent, ReplanningStrategy, compare_replanning_strategies

@dataclass
class PerformanceMetrics:
    """Comprehensive performance metrics for algorithm evaluation."""
    algorithm_name: str
    success_rate: float
    average_path_cost: float
    average_path_length: int
    average_nodes_expanded: int
    average_planning_time: float
    median_planning_time: float
    std_planning_time: float
    fuel_efficiency: float
    scalability_score: float
    robustness_score: float
    memory_usage: float
    cpu_usage: float

@dataclass
class ExperimentResult:
    """Result of a single experiment run."""
    algorithm: str
    map_size: str
    scenario: str
    success: bool
    path_cost: float
    path_length: int
    nodes_expanded: int
    planning_time: float
    fuel_consumed: float
    deliveries_completed: int
    replanning_events: int
    execution_time: float

class PerformanceAnalyzer:
    """Comprehensive performance analysis system."""
    def __init__(self):
        self.experiment_results: List[ExperimentResult] = []
        self.performance_metrics: Dict[str, PerformanceMetrics] = {}
        
    def run_comprehensive_benchmark(self, num_runs: int = 10) -> Dict[str, Any]:
        """Run comprehensive benchmark across all algorithms and maps."""
        print("Running comprehensive performance benchmark...")
        # Define test scenarios
        scenarios = {
            'small': [
                (Position(0, 0), Position(9, 9)),
                (Position(1, 1), Position(8, 8)),
                (Position(2, 2), Position(7, 7))
            ],
            'medium': [
                (Position(0, 0), Position(19, 19)),
                (Position(5, 5), Position(15, 15)),
                (Position(10, 10), Position(18, 18))
            ],
            'large': [
                (Position(0, 0), Position(49, 49)),
                (Position(10, 10), Position(40, 40)),
                (Position(20, 20), Position(30, 30))
            ],
            'dynamic': [
                (Position(0, 0), Position(14, 14)),
                (Position(1, 1), Position(13, 13)),
                (Position(2, 2), Position(12, 12))
            ]
        }
        # Create environments
        environments = {
            'small': create_test_environment_small(),
            'medium': create_test_environment_medium(),
            'large': create_test_environment_large(),
            'dynamic': create_test_environment_dynamic()
        }
        # Test all combinations
        for run in range(num_runs):
            print(f"Run {run + 1}/{num_runs}")
            
            for map_size, env in environments.items():
                for scenario_idx, (start, goal) in enumerate(scenarios[map_size]):
                    # Test all algorithms
                    results = compare_algorithms(env, start, goal)
                    
                    for alg_name, result in results.items():
                        experiment = ExperimentResult(
                            algorithm=alg_name,
                            map_size=map_size,
                            scenario=f"scenario_{scenario_idx + 1}",
                            success=result.success,
                            path_cost=result.cost,
                            path_length=len(result.path),
                            nodes_expanded=result.nodes_expanded,
                            planning_time=result.time_taken,
                            fuel_consumed=result.cost,  # Assuming 1:1 fuel to cost ratio
                            deliveries_completed=1 if result.success else 0,
                            replanning_events=0,  # Single pathfinding, no replanning
                            execution_time=result.time_taken
                        )
                        self.experiment_results.append(experiment)
        # Calculate performance metrics
        self._calculate_performance_metrics()
        return self._generate_analysis_report()
    
    def run_delivery_benchmark(self, num_runs: int = 5) -> Dict[str, Any]:
        """Run benchmark specifically for delivery scenarios."""
        print("Running delivery scenario benchmark...")
        # Create test scenarios with multiple delivery tasks
        delivery_scenarios = {
            'small': [
                [DeliveryTask("p1", Position(1, 1), Position(8, 8), priority=3),
                 DeliveryTask("p2", Position(2, 2), Position(7, 7), priority=2)],
                [DeliveryTask("p1", Position(0, 1), Position(9, 8), priority=2),
                 DeliveryTask("p2", Position(1, 0), Position(8, 9), priority=1),
                 DeliveryTask("p3", Position(2, 2), Position(7, 7), priority=3)]
            ],
            'medium': [
                [DeliveryTask("p1", Position(1, 1), Position(18, 18), priority=3),
                 DeliveryTask("p2", Position(2, 2), Position(17, 17), priority=2),
                 DeliveryTask("p3", Position(3, 3), Position(16, 16), priority=1)]
            ],
            'dynamic': [
                [DeliveryTask("p1", Position(1, 1), Position(13, 13), priority=3),
                 DeliveryTask("p2", Position(2, 2), Position(12, 12), priority=2)]
            ]
        }
        strategies = [
            PlanningStrategy.BFS,
            PlanningStrategy.UNIFORM_COST,
            PlanningStrategy.ASTAR_MANHATTAN,
            PlanningStrategy.ASTAR_EUCLIDEAN,
            PlanningStrategy.ASTAR_DIAGONAL,
            PlanningStrategy.HILL_CLIMBING,
            PlanningStrategy.SIMULATED_ANNEALING
        ]
        environments = {
            'small': create_test_environment_small(),
            'medium': create_test_environment_medium(),
            'dynamic': create_test_environment_dynamic()
        }
        for run in range(num_runs):
            print(f"Delivery run {run + 1}/{num_runs}")
            for map_size, tasks_list in delivery_scenarios.items():
                env = environments[map_size]               
                for scenario_idx, tasks in enumerate(tasks_list):
                    for strategy in strategies:
                        # Create agent
                        agent = DeliveryAgent(env, Position(0, 0), initial_fuel=100.0)
                        agent.set_planning_strategy(strategy)
                        # Add tasks
                        for task in tasks:
                            agent.add_delivery_task(task)
                        # Run simulation
                        start_time = time.time()
                        stats = agent.run_simulation(max_steps=500)
                        execution_time = time.time() - start_time
                                                # Record results
                        experiment = ExperimentResult(
                            algorithm=strategy.value,
                            map_size=map_size,
                            scenario=f"delivery_{scenario_idx + 1}",
                            success=stats['deliveries_completed'] > 0,
                            path_cost=stats['total_fuel_consumed'],
                            path_length=stats['simulation_steps'],
                            nodes_expanded=0,  # Not tracked in delivery simulation
                            planning_time=stats['total_planning_time'],
                            fuel_consumed=stats['total_fuel_consumed'],
                            deliveries_completed=stats['deliveries_completed'],
                            replanning_events=stats['replanning_events'],
                            execution_time=execution_time
                        )
                        self.experiment_results.append(experiment)
        return self._generate_delivery_analysis()
    
    def run_replanning_benchmark(self, num_runs: int = 3) -> Dict[str, Any]:
        """Run benchmark for replanning strategies."""
        print("Running replanning strategy benchmark...")
        # Create dynamic environment
        env = create_test_environment_dynamic()
        # Test scenarios
        tasks = [
            DeliveryTask("urgent", Position(1, 1), Position(13, 13), priority=3),
            DeliveryTask("standard", Position(2, 2), Position(12, 12), priority=2),
            DeliveryTask("low", Position(3, 3), Position(11, 11), priority=1)
        ]
        # Test all replanning strategies
        replanning_strategies = [
            ReplanningStrategy.IMMEDIATE,
            ReplanningStrategy.PREDICTIVE,
            ReplanningStrategy.ADAPTIVE,
            ReplanningStrategy.CONSERVATIVE,
            ReplanningStrategy.AGGRESSIVE
        ]
        for run in range(num_runs):
            print(f"Replanning run {run + 1}/{num_runs}")
            
            for strategy in replanning_strategies:
                # Create agent with replanning strategy
                agent = AdvancedReplanningAgent(env, Position(0, 0), 
                                              initial_fuel=100.0, 
                                              replanning_strategy=strategy)
                
                # Add tasks
                for task in tasks:
                    agent.add_delivery_task(task)
                
                # Run simulation
                start_time = time.time()
                step = 0
                max_steps = 300
                
                while step < max_steps and agent.state.fuel > 0:
                    if not agent.execute_next_action_with_replanning():
                        break
                    step += 1
                    env.advance_time()
                
                execution_time = time.time() - start_time
                replanning_stats = agent.get_replanning_statistics()
                # Record results
                experiment = ExperimentResult(
                    algorithm=f"replanning_{strategy.value}",
                    map_size="dynamic",
                    scenario="multi_delivery",
                    success=agent.stats['deliveries_completed'] > 0,
                    path_cost=agent.stats['total_fuel_consumed'],
                    path_length=step,
                    nodes_expanded=0,
                    planning_time=replanning_stats['average_replan_time'],
                    fuel_consumed=agent.stats['total_fuel_consumed'],
                    deliveries_completed=agent.stats['deliveries_completed'],
                    replanning_events=replanning_stats['total_replanning_events'],
                    execution_time=execution_time
                )
                self.experiment_results.append(experiment)
        return self._generate_replanning_analysis()
    
    def _calculate_performance_metrics(self):
        """Calculate comprehensive performance metrics for each algorithm."""
        algorithms = set(result.algorithm for result in self.experiment_results)
        for algorithm in algorithms:
            alg_results = [r for r in self.experiment_results if r.algorithm == algorithm]
            if not alg_results:
                continue
            # Basic metrics
            success_rate = sum(1 for r in alg_results if r.success) / len(alg_results)
            successful_results = [r for r in alg_results if r.success]
            if successful_results:
                avg_path_cost = statistics.mean(r.path_cost for r in successful_results)
                avg_path_length = statistics.mean(r.path_length for r in successful_results)
                avg_nodes_expanded = statistics.mean(r.nodes_expanded for r in successful_results)
                avg_planning_time = statistics.mean(r.planning_time for r in successful_results)
                median_planning_time = statistics.median(r.planning_time for r in successful_results)
                std_planning_time = statistics.stdev(r.planning_time for r in successful_results) if len(successful_results) > 1 else 0
            else:
                avg_path_cost = 0
                avg_path_length = 0
                avg_nodes_expanded = 0
                avg_planning_time = 0
                median_planning_time = 0
                std_planning_time = 0
            
            # Advanced metrics
            fuel_efficiency = self._calculate_fuel_efficiency(alg_results)
            scalability_score = self._calculate_scalability_score(alg_results)
            robustness_score = self._calculate_robustness_score(alg_results)
            
            # Resource usage (simplified estimates)
            memory_usage = self._estimate_memory_usage(alg_results)
            cpu_usage = self._estimate_cpu_usage(alg_results)
            
            self.performance_metrics[algorithm] = PerformanceMetrics(
                algorithm_name=algorithm,
                success_rate=success_rate,
                average_path_cost=avg_path_cost,
                average_path_length=avg_path_length,
                average_nodes_expanded=avg_nodes_expanded,
                average_planning_time=avg_planning_time,
                median_planning_time=median_planning_time,
                std_planning_time=std_planning_time,
                fuel_efficiency=fuel_efficiency,
                scalability_score=scalability_score,
                robustness_score=robustness_score,
                memory_usage=memory_usage,
                cpu_usage=cpu_usage
            )
    def _calculate_fuel_efficiency(self, results: List[ExperimentResult]) -> float:
        """Calculate fuel efficiency score."""
        if not results:
            return 0.0
        # Compare fuel consumption to optimal (Manhattan distance)
        efficiency_scores = []
        for result in results:
            if result.success and result.map_size in ['small', 'medium', 'large', 'dynamic']:
                # Estimate optimal fuel consumption
                optimal_fuel = self._estimate_optimal_fuel(result.map_size)
                if optimal_fuel > 0:
                    efficiency = optimal_fuel / result.fuel_consumed
                    efficiency_scores.append(min(efficiency, 1.0))  # Cap at 1.0
        return statistics.mean(efficiency_scores) if efficiency_scores else 0.0
    
    def _calculate_scalability_score(self, results: List[ExperimentResult]) -> float:
        """Calculate scalability score based on performance across map sizes."""
        map_performance = defaultdict(list)
        for result in results:
            if result.success:
                # Normalize performance by map size
                if result.map_size == 'small':
                    normalized_time = result.planning_time
                elif result.map_size == 'medium':
                    normalized_time = result.planning_time / 4  # Rough scaling factor
                elif result.map_size == 'large':
                    normalized_time = result.planning_time / 25  # Rough scaling factor
                else:
                    normalized_time = result.planning_time
                map_performance[result.map_size].append(normalized_time)
        # Calculate consistency across map sizes
        if len(map_performance) > 1:
            avg_times = [statistics.mean(times) for times in map_performance.values()]
            consistency = 1.0 - (statistics.stdev(avg_times) / statistics.mean(avg_times)) if avg_times else 0.0
            return max(0.0, consistency)
        return 0.5  # Default score if only one map size
    
    def _calculate_robustness_score(self, results: List[ExperimentResult]) -> float:
        """Calculate robustness score based on success rate and consistency."""
        if not results:
            return 0.0
        success_rate = sum(1 for r in results if r.success) / len(results)
        # Calculate consistency (lower variance in planning time = more robust)
        planning_times = [r.planning_time for r in results if r.success]
        if len(planning_times) > 1:
            consistency = 1.0 - (statistics.stdev(planning_times) / statistics.mean(planning_times))
            consistency = max(0.0, consistency)
        else:
            consistency = 0.5
        
        return (success_rate + consistency) / 2
    
    def _estimate_memory_usage(self, results: List[ExperimentResult]) -> float:
        """Estimate memory usage based on nodes expanded."""
        if not results:
            return 0.0
        avg_nodes = statistics.mean(r.nodes_expanded for r in results)
        # Rough estimate: 1KB per node (simplified)
        return avg_nodes * 1024  # bytes
    def _estimate_cpu_usage(self, results: List[ExperimentResult]) -> float:
        """Estimate CPU usage based on planning time."""
        if not results:
            return 0.0
        avg_time = statistics.mean(r.planning_time for r in results)
        # Rough estimate: CPU usage proportional to planning time
        return min(avg_time * 100, 100.0)  # Cap at 100%
    def _estimate_optimal_fuel(self, map_size: str) -> float:
        """Estimate optimal fuel consumption for map size."""
        estimates = {
            'small': 18.0,    # Rough Manhattan distance for 10x10
            'medium': 38.0,   # Rough Manhattan distance for 20x20
            'large': 98.0,    # Rough Manhattan distance for 50x50
            'dynamic': 28.0   # Rough Manhattan distance for 15x15
        }
        return estimates.get(map_size, 20.0)
    def _generate_analysis_report(self) -> Dict[str, Any]:
        """Generate comprehensive analysis report."""
        report = {
            'summary': {
                'total_experiments': len(self.experiment_results),
                'algorithms_tested': len(self.performance_metrics),
                'successful_experiments': sum(1 for r in self.experiment_results if r.success),
                'overall_success_rate': sum(1 for r in self.experiment_results if r.success) / len(self.experiment_results) if self.experiment_results else 0
            },
            'algorithm_rankings': self._rank_algorithms(),
            'performance_metrics': {name: asdict(metrics) for name, metrics in self.performance_metrics.items()},
            'recommendations': self._generate_recommendations(),
            'detailed_analysis': self._generate_detailed_analysis()
        }
        return report
    
    def _rank_algorithms(self) -> Dict[str, List[str]]:
        """Rank algorithms by different criteria."""
        rankings = {
            'by_success_rate': [],
            'by_efficiency': [],
            'by_speed': [],
            'by_scalability': [],
            'by_robustness': []
        }
        # Sort by different criteria
        rankings['by_success_rate'] = sorted(self.performance_metrics.keys(), 
                                           key=lambda x: self.performance_metrics[x].success_rate, 
                                           reverse=True)
        rankings['by_efficiency'] = sorted(self.performance_metrics.keys(), 
                                         key=lambda x: self.performance_metrics[x].fuel_efficiency, 
                                         reverse=True)
        rankings['by_speed'] = sorted(self.performance_metrics.keys(), 
                                    key=lambda x: self.performance_metrics[x].average_planning_time)
        rankings['by_scalability'] = sorted(self.performance_metrics.keys(), 
                                          key=lambda x: self.performance_metrics[x].scalability_score, 
                                          reverse=True)
        rankings['by_robustness'] = sorted(self.performance_metrics.keys(), 
                                         key=lambda x: self.performance_metrics[x].robustness_score, 
                                         reverse=True)
        return rankings
    
    def _generate_recommendations(self) -> Dict[str, str]:
        """Generate recommendations based on analysis."""
        recommendations = {}
        if self.performance_metrics:
            # Find best overall algorithm
            best_overall = max(self.performance_metrics.keys(), 
                             key=lambda x: (self.performance_metrics[x].success_rate + 
                                          self.performance_metrics[x].fuel_efficiency + 
                                          self.performance_metrics[x].robustness_score) / 3)
            recommendations['best_overall'] = f"Best overall algorithm: {best_overall}"
            # Find best for specific use cases
            best_speed = min(self.performance_metrics.keys(), 
                           key=lambda x: self.performance_metrics[x].average_planning_time)
            recommendations['best_speed'] = f"Fastest algorithm: {best_speed}"
            best_efficiency = max(self.performance_metrics.keys(), 
                                key=lambda x: self.performance_metrics[x].fuel_efficiency)
            recommendations['best_efficiency'] = f"Most fuel-efficient: {best_efficiency}"
            best_robust = max(self.performance_metrics.keys(), 
                            key=lambda x: self.performance_metrics[x].robustness_score)
            recommendations['best_robustness'] = f"Most robust: {best_robust}"
        return recommendations
    
    def _generate_detailed_analysis(self) -> Dict[str, Any]:
        """Generate detailed analysis with insights."""
        analysis = {
            'performance_trends': self._analyze_performance_trends(),
            'bottlenecks': self._identify_bottlenecks(),
            'optimization_opportunities': self._identify_optimization_opportunities(),
            'scalability_analysis': self._analyze_scalability()
        }
        return analysis
    
    def _analyze_performance_trends(self) -> Dict[str, Any]:
        """Analyze performance trends across different scenarios."""
        trends = {}
        # Analyze by map size
        map_performance = defaultdict(list)
        for result in self.experiment_results:
            if result.success:
                map_performance[result.map_size].append(result.planning_time)
        trends['map_size_impact'] = {
            map_size: {
                'avg_planning_time': statistics.mean(times),
                'std_planning_time': statistics.stdev(times) if len(times) > 1 else 0,
                'sample_count': len(times)
            }
            for map_size, times in map_performance.items()
        }
        return trends
    
    def _identify_bottlenecks(self) -> List[str]:
        """Identify performance bottlenecks."""
        bottlenecks = []
        # Check for high planning times
        high_time_algs = [name for name, metrics in self.performance_metrics.items() 
                         if metrics.average_planning_time > 0.1]
        if high_time_algs:
            bottlenecks.append(f"High planning time: {', '.join(high_time_algs)}")
        # Check for low success rates
        low_success_algs = [name for name, metrics in self.performance_metrics.items() 
                           if metrics.success_rate < 0.8]
        if low_success_algs:
            bottlenecks.append(f"Low success rate: {', '.join(low_success_algs)}")
        # Check for high memory usage
        high_memory_algs = [name for name, metrics in self.performance_metrics.items() 
                           if metrics.memory_usage > 1000000]  # 1MB
        if high_memory_algs:
            bottlenecks.append(f"High memory usage: {', '.join(high_memory_algs)}")
        return bottlenecks
    
    def _identify_optimization_opportunities(self) -> List[str]:
        """Identify opportunities for optimization."""
        opportunities = []
        # Check for algorithms with high variance in planning time
        high_variance_algs = [name for name, metrics in self.performance_metrics.items() 
                             if metrics.std_planning_time > metrics.average_planning_time * 0.5]
        if high_variance_algs:
            opportunities.append(f"High variance in planning time: {', '.join(high_variance_algs)}")
        # Check for algorithms with low scalability
        low_scalability_algs = [name for name, metrics in self.performance_metrics.items() 
                               if metrics.scalability_score < 0.5]
        if low_scalability_algs:
            opportunities.append(f"Low scalability: {', '.join(low_scalability_algs)}")
        return opportunities
    
    def _analyze_scalability(self) -> Dict[str, Any]:
        """Analyze scalability across different map sizes."""
        scalability_analysis = {}
        for algorithm in set(r.algorithm for r in self.experiment_results):
            alg_results = [r for r in self.experiment_results if r.algorithm == algorithm and r.success]
            if len(alg_results) < 2:
                continue
            # Group by map size
            map_times = defaultdict(list)
            for result in alg_results:
                map_times[result.map_size].append(result.planning_time)
            # Calculate scaling factors
            if 'small' in map_times and 'medium' in map_times:
                small_avg = statistics.mean(map_times['small'])
                medium_avg = statistics.mean(map_times['medium'])
                scaling_factor = medium_avg / small_avg if small_avg > 0 else 0
                scalability_analysis[algorithm] = {
                    'small_to_medium_scaling': scaling_factor,
                    'scalability_grade': 'Good' if scaling_factor < 10 else 'Poor'
                }
        return scalability_analysis
    
    def _generate_delivery_analysis(self) -> Dict[str, Any]:
        """Generate analysis specific to delivery scenarios."""
        delivery_results = [r for r in self.experiment_results if 'delivery' in r.scenario]
        if not delivery_results:
            return {}
        analysis = {
            'delivery_performance': {},
            'strategy_comparison': {},
            'efficiency_analysis': {}
        }
        # Analyze by algorithm
        for algorithm in set(r.algorithm for r in delivery_results):
            alg_results = [r for r in delivery_results if r.algorithm == algorithm]
            analysis['delivery_performance'][algorithm] = {
                'avg_deliveries_completed': statistics.mean(r.deliveries_completed for r in alg_results),
                'avg_fuel_consumed': statistics.mean(r.fuel_consumed for r in alg_results),
                'avg_replanning_events': statistics.mean(r.replanning_events for r in alg_results),
                'success_rate': sum(1 for r in alg_results if r.success) / len(alg_results)
            }
        return analysis
    
    def _generate_replanning_analysis(self) -> Dict[str, Any]:
        """Generate analysis specific to replanning strategies."""
        replanning_results = [r for r in self.experiment_results if 'replanning' in r.algorithm]
        if not replanning_results:
            return {}
        analysis = {
            'replanning_performance': {},
            'strategy_effectiveness': {},
            'efficiency_comparison': {}
        }
        # Analyze by replanning strategy
        for algorithm in set(r.algorithm for r in replanning_results):
            alg_results = [r for r in replanning_results if r.algorithm == algorithm]
            analysis['replanning_performance'][algorithm] = {
                'avg_deliveries_completed': statistics.mean(r.deliveries_completed for r in alg_results),
                'avg_replanning_events': statistics.mean(r.replanning_events for r in alg_results),
                'avg_execution_time': statistics.mean(r.execution_time for r in alg_results),
                'success_rate': sum(1 for r in alg_results if r.success) / len(alg_results)
            }
        return analysis
    
    def save_results(self, filename: str = 'performance_analysis.json'):
        """Save analysis results to file."""
        report = self._generate_analysis_report()
        with open(filename, 'w') as f:
            json.dump(report, f, indent=2, default=str)
        print(f"Performance analysis saved to {filename}")

    def generate_visualization(self, output_dir: str = 'analysis_plots'):
        """Generate comprehensive visualization plots."""
        import os
        os.makedirs(output_dir, exist_ok=True)
        # Performance comparison plots
        self._plot_algorithm_comparison(output_dir)
        self._plot_scalability_analysis(output_dir)
        self._plot_efficiency_analysis(output_dir)
        self._plot_robustness_analysis(output_dir)
        print(f"Visualization plots saved to {output_dir}/")
    
    def _plot_algorithm_comparison(self, output_dir: str):
        """Plot algorithm comparison charts."""
        if not self.performance_metrics:
            return
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('Algorithm Performance Comparison', fontsize=16, fontweight='bold')
        algorithms = list(self.performance_metrics.keys())
    
        # Success rate
        success_rates = [self.performance_metrics[alg].success_rate for alg in algorithms]
        axes[0, 0].bar(algorithms, success_rates, color='skyblue')
        axes[0, 0].set_title('Success Rate')
        axes[0, 0].set_ylabel('Success Rate')
        axes[0, 0].tick_params(axis='x', rotation=45)
        
        # Planning time
        planning_times = [self.performance_metrics[alg].average_planning_time for alg in algorithms]
        axes[0, 1].bar(algorithms, planning_times, color='lightcoral')
        axes[0, 1].set_title('Average Planning Time')
        axes[0, 1].set_ylabel('Time (seconds)')
        axes[0, 1].tick_params(axis='x', rotation=45)
        
        # Fuel efficiency
        fuel_efficiency = [self.performance_metrics[alg].fuel_efficiency for alg in algorithms]
        axes[1, 0].bar(algorithms, fuel_efficiency, color='lightgreen')
        axes[1, 0].set_title('Fuel Efficiency')
        axes[1, 0].set_ylabel('Efficiency Score')
        axes[1, 0].tick_params(axis='x', rotation=45)
        
        # Robustness
        robustness = [self.performance_metrics[alg].robustness_score for alg in algorithms]
        axes[1, 1].bar(algorithms, robustness, color='gold')
        axes[1, 1].set_title('Robustness Score')
        axes[1, 1].set_ylabel('Robustness Score')
        axes[1, 1].tick_params(axis='x', rotation=45)
        plt.tight_layout()
        plt.savefig(f'{output_dir}/algorithm_comparison.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_scalability_analysis(self, output_dir: str):
        """Plot scalability analysis."""
        # Group results by map size and algorithm
        map_sizes = ['small', 'medium', 'large', 'dynamic']
        algorithms = list(set(r.algorithm for r in self.experiment_results))
        fig, ax = plt.subplots(figsize=(12, 8))
        for algorithm in algorithms:
            times = []
            for map_size in map_sizes:
                alg_results = [r for r in self.experiment_results 
                             if r.algorithm == algorithm and r.map_size == map_size and r.success]
                if alg_results:
                    avg_time = statistics.mean(r.planning_time for r in alg_results)
                    times.append(avg_time)
                else:
                    times.append(0)
            ax.plot(map_sizes, times, marker='o', label=algorithm, linewidth=2)
        
        ax.set_title('Scalability Analysis - Planning Time vs Map Size')
        ax.set_xlabel('Map Size')
        ax.set_ylabel('Average Planning Time (seconds)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(f'{output_dir}/scalability_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_efficiency_analysis(self, output_dir: str):
        """Plot efficiency analysis."""
        if not self.performance_metrics:
            return 
        algorithms = list(self.performance_metrics.keys())
        fuel_efficiency = [self.performance_metrics[alg].fuel_efficiency for alg in algorithms]
        scalability = [self.performance_metrics[alg].scalability_score for alg in algorithms]
        fig, ax = plt.subplots(figsize=(10, 8))
        scatter = ax.scatter(fuel_efficiency, scalability, s=100, alpha=0.7)
        for i, alg in enumerate(algorithms):
            ax.annotate(alg, (fuel_efficiency[i], scalability[i]), 
                       xytext=(5, 5), textcoords='offset points')
        ax.set_xlabel('Fuel Efficiency Score')
        ax.set_ylabel('Scalability Score')
        ax.set_title('Efficiency vs Scalability Trade-off')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(f'{output_dir}/efficiency_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_robustness_analysis(self, output_dir: str):
        """Plot robustness analysis."""
        if not self.performance_metrics:
            return
        
        algorithms = list(self.performance_metrics.keys())
        robustness = [self.performance_metrics[alg].robustness_score for alg in algorithms]
        success_rates = [self.performance_metrics[alg].success_rate for alg in algorithms]
        fig, ax = plt.subplots(figsize=(10, 8))
        bars = ax.bar(algorithms, robustness, color='lightblue', alpha=0.7)
        # Add success rate as text on bars
        for i, (bar, success_rate) in enumerate(zip(bars, success_rates)):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                   f'SR: {success_rate:.2f}', ha='center', va='bottom')
        ax.set_ylabel('Robustness Score')
        ax.set_title('Algorithm Robustness Analysis')
        ax.tick_params(axis='x', rotation=45)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(f'{output_dir}/robustness_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()

def run_comprehensive_analysis():
    """Run comprehensive performance analysis."""
    analyzer = PerformanceAnalyzer()
    print("Starting comprehensive performance analysis...")
    # Run all benchmarks
    print("\n1. Running algorithm benchmark...")
    algorithm_report = analyzer.run_comprehensive_benchmark(num_runs=5)
    print("\n2. Running delivery benchmark...")
    delivery_report = analyzer.run_delivery_benchmark(num_runs=3)
    print("\n3. Running replanning benchmark...")
    replanning_report = analyzer.run_replanning_benchmark(num_runs=2)
    # Generate reports
    print("\n4. Generating analysis reports...")
    analyzer.save_results('comprehensive_analysis.json')
    print("\n5. Generating visualizations...")
    analyzer.generate_visualization('analysis_plots')
    print("\nAnalysis complete!")
    # Print summary
    print("\n" + "="*60)
    print("PERFORMANCE ANALYSIS SUMMARY")
    print("="*60)
    
    if analyzer.performance_metrics:
        best_overall = max(analyzer.performance_metrics.keys(), 
                          key=lambda x: (analyzer.performance_metrics[x].success_rate + 
                                       analyzer.performance_metrics[x].fuel_efficiency + 
                                       analyzer.performance_metrics[x].robustness_score) / 3)
        print(f"Best Overall Algorithm: {best_overall}")
        print(f"Success Rate: {analyzer.performance_metrics[best_overall].success_rate:.2f}")
        print(f"Fuel Efficiency: {analyzer.performance_metrics[best_overall].fuel_efficiency:.2f}")
        print(f"Robustness: {analyzer.performance_metrics[best_overall].robustness_score:.2f}")
    print(f"\nTotal experiments run: {len(analyzer.experiment_results)}")
    print("Analysis files generated:")
    print("  - comprehensive_analysis.json")
    print("  - analysis_plots/ (visualization directory)")

if __name__ == "__main__":
    run_comprehensive_analysis()
