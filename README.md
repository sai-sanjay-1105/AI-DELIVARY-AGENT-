# Delivery-Agent
# Autonomous Delivery Agent System

An autonomous delivery agent that navigates a 2D grid city to deliver packages using various search algorithms and planning strategies. The system demonstrates dynamic replanning capabilities when obstacles appear and provides comprehensive experimental analysis.

## 🚀 Features

- **Multiple Search Algorithms**: BFS, Uniform-cost search, A* (with Manhattan and Euclidean heuristics), Hill-climbing, and Simulated Annealing
- **Dynamic Environment**: Support for static obstacles, varying terrain costs, and moving obstacles
- **Dynamic Replanning**: Automatic replanning when dynamic obstacles block the path
- **Comprehensive Testing**: Multiple test maps and experimental comparison framework
- **CLI Interface**: Easy-to-use command-line interface for running experiments
- **Visualization Tools**: Interactive and static visualizations of agent behavior
- **Performance Optimization**: Caching, hierarchical planning, and memory optimization

## 📋 Requirements

- Python 3.8+
- numpy >= 1.21.0
- matplotlib >= 3.5.0

## 🛠️ Installation

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd autonomous-delivery-agent
   ```

2. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

## 🎯 Quick Start

### 1. Create Test Maps
```bash
python cli.py create-maps
```
This creates four test maps:
- `small`: 10×10 grid with basic obstacles
- `medium`: 20×20 grid with more complex terrain
- `large`: 50×50 grid with multiple building clusters
- `dynamic`: 15×15 grid with moving obstacles

### 2. Compare Algorithms
```bash
python cli.py compare --map small --start 0,0 --goal 9,9
```

### 3. Run Delivery Simulation
```bash
python cli.py simulate --map medium --strategy astar_manhattan --max-steps 500
```

### 4. Dynamic Replanning Demo
```bash
python dynamic_replanning_demo.py
```

## 📖 Usage Guide

### Command Line Interface

The system provides a comprehensive CLI for all operations:

```bash
python cli.py <command> [options]
```

#### Available Commands

1. **create-maps**: Create test environment maps
   ```bash
   python cli.py create-maps
   ```

2. **compare**: Compare all algorithms on a specific route
   ```bash
   python cli.py compare --map <map_name> --start <x,y> --goal <x,y>
   ```

3. **simulate**: Run delivery simulation with specific strategy
   ```bash
   python cli.py simulate --map <map_name> --strategy <strategy> [--max-steps <steps>]
   ```

4. **experiment**: Run comprehensive experiments on all maps
   ```bash
   python cli.py experiment
   ```

5. **analyze**: Generate analysis plots from experiment results
   ```bash
   python cli.py analyze
   ```

#### Available Strategies
- `bfs`: Breadth-First Search
- `uniform_cost`: Uniform-Cost Search
- `astar_manhattan`: A* with Manhattan distance heuristic
- `astar_euclidean`: A* with Euclidean distance heuristic
- `astar_diagonal`: A* with diagonal distance heuristic
- `hill_climbing`: Hill-climbing with random restarts
- `simulated_annealing`: Simulated Annealing

### Dynamic Replanning Demonstration

The system includes a comprehensive demonstration of dynamic replanning:

```bash
python dynamic_replanning_demo.py
```

This will:
- Create a dynamic environment with moving obstacles
- Run an agent with multiple delivery tasks
- Log all replanning events with detailed information
- Generate comprehensive results and statistics

### Visualization Tools

Create visualizations of agent behavior and algorithm performance:

```python
from visualization_tools import AgentVisualizer, create_comprehensive_visualization_report
from environment import create_test_environment_dynamic
from delivery_agent import DeliveryAgent

# Create environment and agent
env = create_test_environment_dynamic()
agent = DeliveryAgent(env, Position(0, 0))

# Create visualizer
visualizer = AgentVisualizer(env, agent)

# Generate environment visualization
fig = visualizer.create_environment_visualization("My Environment")
visualizer.save_visualization(fig, "my_environment.png")
```

## 🏗️ Project Structure

```
├── environment.py              # Grid environment model
├── search_algorithms.py        # Search algorithm implementations
├── delivery_agent.py           # Delivery agent with planning strategies
├── optimization.py             # Performance optimization utilities
├── cli.py                      # Command-line interface
├── dynamic_replanning_demo.py  # Dynamic replanning demonstration
├── visualization_tools.py      # Visualization and analysis tools
├── test_system.py             # Comprehensive test suite
├── requirements.txt           # Python dependencies
├── README.md                  # This file
├── COMPREHENSIVE_REPORT.md    # Detailed technical report
├── DEMO_VIDEO_SCRIPT.md       # Demo video script
└── test_maps/                # Generated test maps
    ├── small_map.json
    ├── medium_map.json
    ├── large_map.json
    └── dynamic_map.json
```

## 🔬 Algorithm Details

### Uninformed Search
- **BFS**: Explores all nodes at current depth before moving to next level
- **Uniform-Cost Search**: Explores nodes in order of increasing path cost

### Informed Search
- **A***: Uses heuristic function to guide search toward goal
  - Manhattan distance: `|x₁-x₂| + |y₁-y₂|` (admissible for 4-connected grid)
  - Euclidean distance: `√((x₁-x₂)² + (y₁-y₂)²)` (admissible but less informed)
  - Diagonal distance: `max(dx, dy) + (1.414-1) × min(dx, dy)` (for 8-connected grids)

### Local Search
- **Hill-Climbing**: Greedy local search with random restarts
- **Simulated Annealing**: Probabilistic local search with temperature cooling

## 🌍 Environment Model

### Grid Representation
- 2D grid with integer coordinates
- Each cell has a terrain type with associated movement cost
- 4-connected movement (up, down, left, right)

### Terrain Types and Costs
| Terrain | Cost | Description |
|---------|------|-------------|
| Road | 1 | Fastest movement path |
| Grass | 2 | Moderate difficulty |
| Water | 3 | Difficult terrain |
| Mountain | 4 | Very difficult terrain |
| Building | ∞ | Impassable obstacles |

### Dynamic Obstacles
- Moving obstacles with deterministic schedules
- Agent can plan knowing future obstacle positions
- Automatic replanning when obstacles block path

## 📊 Experimental Results

The system includes comprehensive experimental analysis comparing algorithms across different map sizes and scenarios.

### Key Findings

1. **A* with Manhattan Distance**: Best overall performance for most scenarios
2. **Dynamic Replanning**: 100% success rate in handling moving obstacles
3. **Scalability**: Consistent performance across different map sizes
4. **Fuel Efficiency**: Optimal fuel usage maintained despite replanning

### Performance Metrics
- Success rate
- Path cost
- Nodes expanded
- Execution time
- Replanning events

## 🧪 Testing

Run the comprehensive test suite:

```bash
python -m unittest test_system -v
```

The test suite covers:
- Environment functionality
- Search algorithms
- Delivery agent behavior
- Dynamic replanning
- Integration tests

## 📈 Performance Optimization

The system includes several optimization features:

### Path Caching
- Intelligent caching of frequently used paths
- Configurable cache size and eviction policies
- Significant performance improvement for repeated queries

### Hierarchical Planning
- Multi-level planning for large maps
- Cluster-based pathfinding
- Reduced computational complexity

### Memory Optimization
- Memory-efficient agent implementation
- Path history management
- Configurable memory limits

## 🎥 Demo and Screenshots

The project includes a comprehensive demo video script (`DEMO_VIDEO_SCRIPT.md`) that provides:

1. **Project Overview** (30s)
2. **Environment Setup** (30s)
3. **Algorithm Comparison** (90s)
4. **Delivery Simulation** (60s)
5. **Dynamic Replanning** (60s)
6. **Comprehensive Analysis** (30s)

Total runtime: 5 minutes

## 📚 Documentation

- **COMPREHENSIVE_REPORT.md**: Detailed technical report with experimental results
- **DEMO_VIDEO_SCRIPT.md**: Step-by-step demo script with screenshots
- **Code Documentation**: Comprehensive docstrings and comments throughout

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- Pathfinding algorithms based on classical AI search methods
- Dynamic replanning inspired by robotics and autonomous vehicle research
- Visualization tools using matplotlib and numpy

## 📞 Support

For questions, issues, or contributions:
- Create an issue in the repository
- Check the comprehensive report for detailed analysis
- Review the demo script for usage examples

---

**Note**: This system is designed for educational and research purposes, demonstrating autonomous agent behavior, pathfinding algorithms, and dynamic replanning capabilities in simulated environments.