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

## note
