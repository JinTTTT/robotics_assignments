# RRT Motion Planning for PUMA Robot

## Project Overview

This project implements an enhanced Rapidly-Exploring Random Tree (RRT) motion planner for the PUMA robot to navigate an L-shaped gibbet end-effector through a constrained environment. The implementation is based on the Robotics Library (RobLib) and extends the basic RRT-Connect algorithm with custom optimizations.

## Problem Description

The goal is to plan a collision-free path for a PUMA robot manipulator to maneuver an L-shaped end-effector through a small hole. The planner computes a trajectory that will be executed on the real robot hardware.

## Installation and Setup

### Prerequisites

- Robotics Library (RobLib) - [http://www.roboticslibrary.org/](http://www.roboticslibrary.org/)
- Follow the installation instructions provided in the tutorial slides and videos

### Building the Project

```bash
cd <folder_where_you_installed_tutorialPlan>/build
cmake ..
make
```

### Running the Planner

```bash
./tutorialPlan
```

Press **space** to start the planning process. The planner will:
1. Generate a search tree
2. Visualize the planning process
3. Find a collision-free path
4. Optimize the path using the built-in optimizer

## Implementation

### Core Files

- **OurPlanner.h / OurPlanner.cpp** - Custom RRT planner implementation
- **OurSampler.cpp** - Custom sampling strategy (if modified)
- **TutorialPlanSystem.cpp** - Main system for testing different planners

### RRT Extensions Implemented

Our planner extends the bidirectional RRT-Connect (RrtConConBase) algorithm with the following enhancements:

1. **[Extension 1 Name]**
    - Description of the modification
    - Rationale for the improvement

2. **[Extension 2 Name]**
    - Description of the modification
    - Rationale for the improvement

3. **[Extension 3 Name]**
    - Description of the modification
    - Rationale for the improvement

## Performance Evaluation

### Benchmark Setup

The planner was evaluated against the baseline RRTConCon algorithm using:
- 10 runs per configuration
- Two scenarios: forward (start → goal) and reversed (goal → start)
- Metrics: average time, standard deviation, nodes, and collision queries

### Results

| Metric | RRTConCon | RRTConCon (reversed) | Our Planner | Our Planner (reversed) |
|--------|-----------|----------------------|--------------|-------------------------|
| avgT | - | - | - | - |
| stdT | - | - | - | - |
| avgNodes | - | - | - | - |
| avgQueries | - | - | - | - |

*Results stored in `benchmark.csv`*

## Output Files

### trajectory.txt

The planned trajectory is saved in the following format:

```
q1(t1) q2(t1) q3(t1) q4(t1) q5(t1) q6(t1)
q1(t2) q2(t2) q3(t2) q4(t2) q5(t2) q6(t2)
...
```

Where:
- `qi(tj)` = angle (in radians) of joint *i* at time step *j*
- Path goes forward (initial → goal) then backward (goal → initial)
- Ready for execution on the real PUMA robot

## Project Structure

```
[To be added: Project directory structure and file descriptions]
```

## Testing Different Planners

You can test various planners included in RobLib by modifying `TutorialPlanSystem.cpp`:

- **Rrt** - Basic RRT
- **RrtConCon** - Bidirectional RRT-Connect
- **RrtExtExt** - Extended variant
- **OurPlanner** - Custom implementation

## Important Notes

- The interface to the planner class should **not** be modified
- Testing is performed with the default `TutorialPlanSystem.cpp`
- All code should be well-commented
- Do not add additional scripts to the project

## Documentation

For detailed information about the extensions, implementation details, and performance analysis, please refer to the technical report included in the submission.

## References

- Robotics Library: [http://www.roboticslibrary.org/](http://www.roboticslibrary.org/)
- RRT Algorithm: LaValle, S. M. (1998). "Rapidly-exploring random trees: A new tool for path planning"
- RRT-Connect: Kuffner, J. J., & LaValle, S. M. (2000). "RRT-connect: An efficient approach to single-query path planning"

## License

This project is part of TU Berlin Robotics Lab Assignment #5.

---

**Course:** TU Berlin Robotics  
**Assignment:** Lab Assignment #5  
**Topic:** Motion Planning with RRT