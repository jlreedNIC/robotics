# Assignment 2: Create3 Takes a Random Walk
Using ROS2 on Create3.

From your dock:
1. Straight out 1.0m
2. Do a random walk of 0.75m. Each walk should have random direction, although you can limit your turns if you want.
3. If you run into something(the bumper signal on the front of the robot), turn 180 degrees, move 0.5m and then go back into your random walk.
4. Keep track of your x/y position as you go through the moves.
5. After 10 random walks(not including bump responses) return to the vicinity of the dock and commence docking.

#### Additional Requirements
1. Write-up including a block diagram

##### Block diagram(including algorithm discussion - return to base, etc) presented to class by each person on Feb 7th

## Due dates
Demo: Tuesday 2/28
Write-Up: Wednesday 2/22? through email to Jacob

## Write-Up requirements
Put block diagram at the top. Describe in detail each step of the program and how it works. Carefully describe how you're responding to bumper responses. Also when you're using callbacks.

### Hints
NO POLLING
