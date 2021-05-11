# multi_arm_coordination

This program implements a coordination method for multiple robot arms (Franka Panda).

Multiple robot arms work in a shared workspace. A target is assigned to each robot. Collision checkings are performed and coordinated trajectories leading all the robots to their targets concurrently without collision are generated with three trajectory replanning strategies:
(1) offline replanning: proportionally slowing down;
(2) offline replanning: delaying;
(3) online replanning: updating temporary goals according to current motion state.


## Instruction
### 1. Run the program in simulation

Open up four terminals and run the following commands.

1.1 Offline replanning
```
$ roslaunch franka_coord coord_offline.launch
$ rosrun franka_coord coord_offline_all
$ roslaunch franka_coord control_offline.launch
$ roslaunch franka_coord give_goals.launch
```
1.2 Online replanning
```
$ roslaunch franka_coord coord_online.launch
$ rosrun franka_coord coord_online_all
$ roslaunch franka_coord control_online.launch
$ roslaunch franka_coord give_goals.launch
```

### 2. Run the program on real robots (Franka Panda)

Run the program on n (n>=2) Franka Panda robots
- n computers (PC1, PC2, PC3, ...), each connect to a robot. PC 1 is set as the master. The coordinators and goal assignments are performed on PC1. The state reading and controller are performed on each PC.
- modify the variable robot_num in coord_offline_all.cpp and coord_online_all.cpp to set the robot number
- modify the robot IP address in control_offline_rm_rx.cpp

2.1 Offline replanning

PC 1
```
$ rosrun franka_coord control_offline_rm_r1
$ rosrun franka_coord read_state_r1
$ roslaunch franka_coord coord_offline_rm.launch
$ rosrun franka_coord coord_offline_all
$ roslaunch franka_coord give_goals_r1.launch
$ roslaunch franka_coord give_goals_r2.launch
```
PC 2
```
$ rosrun franka_coord control_offline_rm_r2
$ rosrun franka_coord read_state_r2
```

2.2 Online replanning

PC 1
```
$ rosrun franka_coord control_online_rm_r1
$ rosrun franka_coord read_state_r1
$ roslaunch franka_coord coord_online_rm.launch
$ rosrun franka_coord coord_online_all
$ roslaunch franka_coord give_goals_r1.launch
$ roslaunch franka_coord give_goals_r2.launch
```
PC 2
```
$ rosrun franka_coord control_offline_rm_r2
$ rosrun franka_coord read_state_r2
```
