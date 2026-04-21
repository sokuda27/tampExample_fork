# Simple TAMP Example

This is has two examples of solving simple pick-and-place problems with the Franka Emika Panda in pybullet. We use [PDDLstream](https://github.com/caelan/pddlstream) as the task and motion planner (TAMP) and use [pb_robot](https://github.com/rachelholladay/pb_robot) as pybullet wrapper. The main example, simple blocks, is essentially a modified version of an this [example within PDDLstream](https://github.com/caelan/pddlstream/tree/stable/examples/pybullet/kuka). This is the recommended example to get started with. There is a second example, stacking, which was developed by Michael Noseworthy. 

## Installation

There are two ways to install this repo. If you already have a catkin workspace (instructions [in this README](https://github.com/rachelholladay/pb_robot)) then you can simply clone this repo in the workspace. If you would like to use this repo with a catkin workspace, the following installation instructions should help: 

1. Install [pb_robot](https://github.com/rachelholladay/pb_robot) using the instructions below, not the ones in the repo's README
    1. Install pb_robot dependencies
        1. ```pip2 install numpy pybullet recordclass catkin_pkg IPython networkx```
        2. ```pip2 install git+https://github.com/rachelholladay/tsr.git```
    2. Clone pb_robot
    3. Compile the IKFast library for the panda
        1. ```cd pb_robot/src/pb_robot/ikfast/franka_panda```
        2. ```python setup.py build```
2. Install [pddlstream](https://github.com/caelan/pddlstream) 
    1. follow installation instructions there
3. Install tampExample (this repo)
    1. Clone tampExample
4. Create a symlink to required repos (this assumes you cloned pb_robot and pddl_stream to your home directory)
    1. ```ln -s ~/pb_robot/src/pb_robot .```
    2. ```ln -s ~/pddlstream/pddlstream .```

## Usage

For the simple blocks example: 

```
cd tampExample/settings/simple_block
python run.py
```

For the stacking example:
```
cd tampExample/settings/stacking
python run_stacking.py
```

`run.py` sets up the problem, calls the planner and then executes the plan (if one was found). The scripts folder also has the two key pddl files. `domain.pddl` defines the predicates and the actions. `stream.pddl` defines the streams, which generate values to satify the actions. The various streams are implemented in `src/tampExample/primitives.py`. Within these streams, for this pick and place example, we opt to define grasp sets via TSRs and execute path planning with a bi-directional RRT. These are not significant (truly, they were made out of convenience) and thus could be swapped with any grasp set definition and path planner.  
