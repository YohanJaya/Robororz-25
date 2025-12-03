# Robororz-25

The robot must start by moving forward from its initial position.
It must then navigate the maze column by column, ensuring it
covers every column sequentially from left to right until it
reaches the last column.
The red arrow in front of the robot indicates the positive Y-axis
(forward direction), and the arrow pointing left marks the
positive X-axis.
The maze grid is divided into square cells, each representing a
unique (X, Y) coordinate.
Green squares in the maze represent checkpoints. The robot
must detect and acknowledge these.
When a checkpoint is detected, the robot must turn on an LED
as a visual indication.
Each checkpoint's X and Y coordinate values are to be
individually added up as the robot progresses.
After completing the maze, the mod value of all the x
coordinates and the mod value of all the y coordinates should
be calculated and compute the final coordinates as:
Final X = (Sum of all checkpoint X values) mod 12
Final Y = (Sum of all checkpoint Y values) mod 12
The robot must then navigate to the cell corresponding to
(Final X, Final Y) and light an LED to signal task completion.
