# TeamHunt

## PSR assignment 3
###### Students:
- Manuel Alberto Silva Gomes
- José Pedro Moura Costa Pinto
- Pedro Miguel Durães De Carvalho
- Gabriele Micheli

## Play description

The play starts with three teams: **red**, **green** and **blue**. Every team has 3 players (ex. **R1**, **R2** and **R3** for the read team). \
Every player can hunt another player and be also be hunted: 
- **Red** hunt **green** and flee from **blue**
- **Green** hunt **blue** and flee from **red**
- **Blue** hunt **red** and flee from **green** 

The hunter catches the prey when the robots are in collision. The goal of the play is to get the maximum team score.  

###### Arena description
The game's arena consists of a football field but with a fence around it that prevents robots from escaping their hunters by always moving in the same direction. 

There are three versions of the arena: 
- **th_arena_1**
- **th_arena_2**
- **th_arena_3**

The last two have walls that turn the arena into a labyrinth.

###### The referee
Since it is always necessary to have someone who can be insulted freely to check that the rules of the game are being followed, it is necessary the presence of a referee. The referee basically checks if there's collision between a hunter and a prey and deducts points from the prey's team.

P.S. The referee is given already developed from @miguelriemoliveira.

For further details, see: [Part 14 (TeamHunt)](https://github.com/miguelriemoliveira/psr_21-22/tree/main/Parte14).

## How to run the program? 
###### STEP 1: Create the empty environment
First of all, launch the empty environment in gazebo, using the command: \
```roslaunch p_group8_bringup gazebo.launch```

It's possible to change the arena scenario, using the argument **world**. \
For example, the command:
```roslaunch p_group8_bringup gazebo.launch world:=arena_1```
create the **th_arena_1** environemnt (see **Arena description** chapter)

###### STEP 2: Create the multi-robot environment
To create the multirobot environment, run the command: \
```roslaunch p_group8_bringup game_bringup.launch world:=arena_1```\
Be certain that the world argument is the same as the one used on step 1.
###### STEP 2a: Create a single robot
To create a single robot instead of a multiple-robot environment, run the command: \
```roslaunch p_group8_bringup bringup.launch player_name:=p_randomName```
###### STEP 3: 'Activate' the referee
To start the referee node, run the command: \
```rosrun th_referee th_referee```
###### STEP 4 (extra): Activate the chat-terminal-bot
If you want to have some fun during the game, open a new terminal and run the command: \
```rosrun p_pgroup8_player chatting.py``` \
In this way you can see the robot's chat during the game, enjoy!
## rviz  
To open the initial **rviz** setup, run the command: \
```roslaunch p_group8_bringup visualize.launch```  

To open the visualization from the navigation toolbox, use the command: \
```roslaunch p_group8_nav visualize.launch```

## Robot manual moving 
###### Teleop
Run the **teleop.launch** file to move the robot using the computer's keyboard. To do this, run the command: \

```roslaunch p_group8_bringup teleop.launch``` 

###### Teleop-joy
Using the [joy](http://wiki.ros.org/joy) package it's possible to control the car using a videogame controller.
To run this code run the command:

```roslaunch p_group8_bringup joy_teleop.launch```

To create the FNR track, use the:

```roslaunch p_group8_bringup fnr_gazebo.launch```

To spawn a regular robot run:

```roslaunch p_group8_bringup bringup.launch```

Here are the buttons used and their functionalities:
- **Left stick**: controls angular velocity
- **A button**: defines linear velocity as 1;
- **B button**: defines linear velocity as 0;
- **X button**: adds 0.1 to linear velocity;
- **Y button**: decreases 0.1 to linear velocity;
- **Right trigger**: temporarily adds 0.5 to linear velocity;

You can see a video demo [here](https://www.youtube.com/watch?v=CBHNlbpLpZM/)!
