
 --------------------------------
Input file format:             |
--------------------------------
First 3 values in rows are Euler angles and the remaining values are position coordinates.

datapoint1: theta1 theta2 theta3 x y z
datapoint2:
datapoint3:
....and so on

Note: "datapoint#" text in not included in output file.

--------------------------------
Output file format:             |
--------------------------------
7 values each corresponding to joint angle.

datapoint1: q1 q2 q3 q4 q5 q6 q7
datapoint2:
datapoint3:
....and so on

Note: "datapoint#" text in not included in input file, it is just used here for demonstration purpose.

--------------------------------
Stats file format:             |
--------------------------------

Input min values
Input max values
Output min values
Output max values