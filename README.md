# TeamHunt

## PSR: assignment 3
###### Students:
- Manuel Alberto Silva Gomes
- Jose Pedro Moura Costa Pinto
- Pedro Miguel Duares De Carvalho
- Gabriele Micheli

## Play description

The play starts with three teams: red, green and blue. Every team has 3 players (ex. **R1**, **R2** and **R3** for the read team). \
Every team can hunt another player and be also be hunted: 
- **Red** hunt **green** and flee from **blue**
- **Green** hunt **blue** and flee from **red**
- **Blue** hunt **red** and flee from **green** 

The hunter catch the prey when the robots are in contact (collision). The goal of the play is to get the maximum team score.  

###### Arena description
The game's arena consists of a football field but with a fence around it that prevents robots from escaping their hunters by always moving in the same direction. 

There are three versions of the arena: 
- **th_arena_1**
- **th_arena_2**
- **th_arena_3**

The last two have walls that turn the arena into a labyrinth.

###### The referee
Since it is always necessary to have someone who can be insulted freely to check that the rules of the game are being followed, it is necessary the presence of a referee. The referee basically checks if there's tcollision between a hunter and a prey and deducts points from the prey's team.

P.S. The referee is given already developed from @miguelriemoliveira.

For further details, see: [Part 14 (TeamHunt)](https://github.com/miguelriemoliveira/psr_21-22/tree/main/Parte14).

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
###### STEP 3: Robot moving
To start the hunting and see the robot moving automatically, run the command: \
```rosrun p_group8_player driver.py```
## rviz  
To open the initial **rviz** setup, run the command: \
```roslaunch p_group8_bringup visualize.launch```  

## Robot manual moving 
###### Teleop
Run the **teleop.launch** file to move the robot using the computer's keyboard. To do this, run the command: \
```roslaunch p_group8_bringup teleop.launch``` 
###### Teleop-joy
If you prefer move the robot using a joystick, this is possible running the command: \
```roslaunch p_group8_bringup joy_teleop.launch```

**GO AHEAD...**
