## Setup
Use ubuntu 20.04 as base env
1. Install RobLib
```
sudo add-apt-repository ppa:roblib/ppa
sudo apt-get update
sudo apt-get install librl-dev
```
2. Build
```
cd tutorialPlan
mkdir build
cd build
cmake ..
make
```
3. Run
```
cd build
./tutorialPlan
```

## Result
explain of the template result: 
"2026-02-06,17:51:31.090,true,RrtConConBase,14906,726146,695434,26295.2"
which means: date, time, solved, planner name, vertices, collision queries, free queries, running time

### Evaluation
we did 20 runs for each planner, 10 runs from start to goal, 10 runs from goal to start.
To reverse the start and goal, we changed code in TutorialPlanSystem.cpp, line 36 and line 39, swap this->goal and this->start.
For evaluation, we need 4 measurements: avgT, stdT, avgNodes, avgQueries

avgT = average running time
stdT = standard deviation of running time
avgNodes = average number of nodes
avgQueries = average number of collision queries


For Baseline: RrtConConBase
| Planner | avgT | stdT | avgNodes | avgQueries |
|---------|------|------|----------|------------|
| RrtConConBase | 18.70 | 6.79 | 10726 | 542079 |
| RrtConConBase_reversed | 19.28 | 7.77 | 11048 | 557666 |

- Both direction perform roughly same for RRTConConBase
- High variance, planner not stable
- Many nodes, many queries
