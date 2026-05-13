# Week 6 - Path Planning

The goal of this assignment is to implement a working path planning algorithm. Your submitted solution should be able to plan a path between any starting and goal position (if a solution exists).

## Task 1

Choose and implement an algorithm which finds a path between the starting position of the robot and the goal position in the map. The path must not collide with any wall. Use the occupancy grid stored in the variable `read_only_vars.discrete_map.map`.

## Task 2

Adjust the planning algorithm to keep a clearance from obstacles. You may use any method. The clearance needs to be at least 0.2 m.

## Task 3

Apply a smoothing algorithm to the generated paths. Use the iterative algorithm provided in the lecture or find another viable method. Discuss the effects of the algorithm's parameters.

## Submission

Send the link to your GitHub repository to the lecturer’s email by **Monday at 23:59 next week**. The repository must contain the unmodified simulator, including the `algorithms` directory with your solution. Include your report in the `report` directory. To ensure easy identification, please tag the final version with the `week_6` tag, or include your solution in a branch named `week_6`.