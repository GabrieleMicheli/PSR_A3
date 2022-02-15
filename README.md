# TeamHunt

## PSR: assignment 3
###### Students:
- Manuel Alberto Silva Gomes
- Jose Pedro Moura Costa Pinto
- Pedro Miguel Duares De Carvalho
- Gabriele Micheli


## How to run the program? 
###### STEP 1: Create the empty environment
First of all, launch the empty environment in gazebo, using the command: \
```roslaunch p_group8_bringup gazebo.launch``` 
###### STEP 2: Create the multi-robot environment
To create the multirobot environment, run the command: \
```roslaunch p_group8_bringup game_bringup.launch```
###### STEP 2a: Create a single robot
To create a single robot instead of a multiple-robot environment, run the command: \
```roslaunch p_group8_bringup bringup.launch player_name:=p_randomName```
## rviz  
To open the initial **rviz** setup, run the command: \
```roslaunch p_group8_bringup visualize.launch```

## Robot moving 
###### Teleop
Run the **teleop.launch** file to move the robot using the computer's keyboard. To do this, run the command: \
```roslaunch p_group8_bringup teleop.launch``` 
###### Teleop-joy
If you prefer move the robot using a joystick, this is possible running the command: \
```roslaunch p_group8_bringup joy_teleop.launch```

**GO AHEAD...**
