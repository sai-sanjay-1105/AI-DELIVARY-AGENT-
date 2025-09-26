# Demo Video Script - Autonomous Delivery Agent System

## Overview
This script provides a 5-minute demonstration of the autonomous delivery agent system, showcasing dynamic replanning capabilities and algorithm performance comparison.

## Video Structure (5 minutes total)

### 1. Introduction (0:00 - 0:30)
**Screenshot 1: Project Overview**
- Show project directory structure
- Highlight key files: `cli.py`, `environment.py`, `delivery_agent.py`
- Display README.md with project description

**Narration:**
"This is an autonomous delivery agent system that navigates a 2D grid city to deliver packages. The system implements multiple pathfinding algorithms and demonstrates dynamic replanning when obstacles appear."

### 2. Environment Setup (0:30 - 1:00)
**Screenshot 2: Environment Creation**
```bash
python cli.py create-maps
```
- Show command execution
- Display created map files in test_maps directory
- Show map file contents (JSON format)

**Screenshot 3: Environment Visualization**
- Run visualization tool showing grid environment
- Point out different terrain types (roads, buildings, water, grass)
- Show dynamic obstacles (moving vehicles)

**Narration:**
"The system creates test environments with varying terrain costs and moving obstacles. Each grid cell has different movement costs, and dynamic obstacles follow deterministic schedules."

### 3. Algorithm Comparison (1:00 - 2:30)
**Screenshot 4: Algorithm Comparison Command**
```bash
python cli.py compare --map small --start 0,0 --goal 9,9
```

**Screenshot 5: Algorithm Results Table**
- Show the comparison results table
- Highlight different algorithms: BFS, A*, Hill Climbing, Simulated Annealing
- Point out success rates, path costs, execution times

**Screenshot 6: Visualization of Different Paths**
- Show visualization comparing different algorithm paths
- Highlight how different algorithms find different routes
- Show performance metrics

**Narration:**
"We can compare different pathfinding algorithms on the same problem. Notice how A* with Manhattan distance finds optimal paths efficiently, while local search methods may find different solutions. Each algorithm has different trade-offs between solution quality and execution time."

### 4. Delivery Simulation (2:30 - 3:30)
**Screenshot 7: Delivery Simulation Setup**
```bash
python cli.py simulate --map medium --strategy astar_manhattan --max-steps 100
```

**Screenshot 8: Agent Movement Visualization**
- Show agent moving through environment
- Display agent picking up packages
- Show agent delivering packages
- Display fuel consumption and statistics

**Narration:**
"The delivery agent can handle multiple tasks, picking up and delivering packages while managing fuel constraints. The agent uses intelligent planning to prioritize tasks and optimize delivery routes."

### 5. Dynamic Replanning Demonstration (3:30 - 4:30)
**Screenshot 9: Dynamic Replanning Demo**
```bash
python dynamic_replanning_demo.py
```

**Screenshot 10: Environment Grid with Moving Obstacles**
- Show the environment grid with agent (A), moving obstacles (O), pickup points (P)
- Demonstrate obstacle movement over time
- Show agent path planning

**Screenshot 11: Replanning Event Log**
- Display the detailed log showing replanning events
- Show timestamps, agent positions, obstacle positions
- Highlight when replanning occurs

**Screenshot 12: Replanning Results**
- Show final statistics including replanning events
- Display successful task completion despite obstacles
- Show performance metrics

**Narration:**
"When moving obstacles block the agent's path, the system automatically detects this and replans a new route. This demonstrates the system's ability to handle dynamic environments in real-time."

### 6. Comprehensive Analysis (4:30 - 5:00)
**Screenshot 13: Experiment Results**
```bash
python cli.py experiment
python cli.py analyze
```

**Screenshot 14: Performance Charts**
- Show generated performance comparison charts
- Display algorithm success rates by map size
- Show path cost comparisons

**Screenshot 15: Project Summary**
- Show final project structure
- Display all generated files and results
- Show comprehensive report

**Narration:**
"The system includes comprehensive testing and analysis tools. We can run experiments across multiple maps and generate detailed performance reports. This demonstrates the system's robustness and provides insights into algorithm performance under different conditions."

## Key Screenshots to Capture

### Screenshot 1: Project Structure
```
Stock Market Predictor/
├── cli.py
├── environment.py
├── delivery_agent.py
├── search_algorithms.py
├── dynamic_replanning_demo.py
├── visualization_tools.py
├── test_maps/
└── README.md
```

### Screenshot 2: Map Creation
```
PS D:\Coding\Projects\Work\Stock Market Predictor> python cli.py create-maps
Created small map: test_maps/small_map.json
Created medium map: test_maps/medium_map.json
Created large map: test_maps/large_map.json
Created dynamic map: test_maps/dynamic_map.json
```

### Screenshot 3: Algorithm Comparison Results
```
Algorithm Comparison Results:
--------------------------------------------------------------------------------
Algorithm            Success  Path Length  Cost       Nodes    Time (s)
--------------------------------------------------------------------------------
BFS                  Yes      19           23.00      450      0.0013
Uniform Cost         Yes      19           23.00      623      0.0059
A* Manhattan         Yes      19           23.00      194      0.0010
A* Euclidean         Yes      19           23.00      263      0.0010
Hill Climbing        Yes      27           34.00      27       0.0371
Simulated Annealing  Yes      2            3.00       1540     0.1193

Best performing algorithm: Simulated Annealing (cost: 3.00, time: 0.1193s)
```

### Screenshot 4: Dynamic Environment Grid
```
=== Step 0 - Environment Grid ===
A..............
.O.............
..P.....O......
..O............
....P..........
.....###.......
.....#P#.......
.....###.......
...............
...............
..........###..
..........###..
..........###..
...............
...............
Legend: A=Agent, O=Moving Obstacle, P=Pickup, D=Delivery, .=Road, g=Grass, ~=Water, ^=Mountain, #=Building
```

### Screenshot 5: Replanning Event Log
```
2025-09-14 21:09:34,251 - WARNING - REPLANNING EVENT DETECTED at step 15!
2025-09-14 21:09:34,251 - WARNING -   Agent was blocked by dynamic obstacle
2025-09-14 21:09:34,251 - WARNING -   Agent position: Position(x=8, y=5)
2025-09-14 21:09:34,251 - WARNING -   Dynamic obstacles: [(10, 5), (8, 7), (9, 6)]
2025-09-14 21:09:34,251 - WARNING -   Total replanning events: 1
```

### Screenshot 6: Final Statistics
```
DYNAMIC REPLANNING DEMONSTRATION COMPLETED
================================================================================
Simulation completed after 45 steps
Final agent position: Position(x=12, y=12)
Final fuel remaining: 67.50
Deliveries completed: 3
Total distance traveled: 32.50
Total fuel consumed: 32.50
Total planning time: 0.0123s
Replanning events: 2
```

## Commands to Run for Screenshots

1. **Project Overview:**
   ```bash
   dir
   type README.md
   ```

2. **Create Maps:**
   ```bash
   python cli.py create-maps
   ```

3. **Algorithm Comparison:**
   ```bash
   python cli.py compare --map small --start 0,0 --goal 9,9
   ```

4. **Delivery Simulation:**
   ```bash
   python cli.py simulate --map medium --strategy astar_manhattan --max-steps 50
   ```

5. **Dynamic Replanning:**
   ```bash
   python dynamic_replanning_demo.py
   ```

6. **Comprehensive Experiment:**
   ```bash
   python cli.py experiment
   python cli.py analyze
   ```

## Recording Tips

1. **Use Clear Fonts**: Set terminal font size to at least 12pt for readability
2. **Highlight Key Information**: Use mouse cursor to point out important results
3. **Show Command Execution**: Let commands run completely before moving to next
4. **Pause for Reading**: Give time to read output tables and logs
5. **Use Annotations**: Add text overlays for key concepts and results

## Expected Runtime
- Total demonstration: 5 minutes
- Each section: 30-60 seconds
- Allow time for commands to complete
- Include brief pauses for audience comprehension

This script provides a comprehensive demonstration of all system capabilities while staying within the 5-minute time limit.
