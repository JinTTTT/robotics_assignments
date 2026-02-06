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
for evaluation, we need 4 measurements: avgT, stdT, avgNodes, avgQueries

avgT = average running time
stdT = standard deviation of running time
avgNodes = average number of nodes
avgQueries = average number of collision queries


For Baseline: RrtConConBase
| Planner | avgT | stdT | avgNodes | avgQueries |
|---------|------|------|----------|------------|
| RrtConConBase | 26295.2 | 1000 | 14906 | 726146 |
| RrtConConBase_reversed | 26295.2 | 1000 | 14906 | 726146 |

