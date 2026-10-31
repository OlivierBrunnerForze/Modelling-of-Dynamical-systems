# Modelling-of-Dynamical-systems
MATLAB code repository for the course SC42155 Modelling of Dynamical Systems.\
Group 23: Olivier Brunner (5552222) & Philip Visser (5838517)

Model of a CoreXY 3D printer: motor and print head kinematics/dynamics, heated nozzle, heat transfer in printed filament, and a hybrid controller for filament replacement and corner-to-corner printing.

Needs Symbolic Math and Control System Toolboxes.

## Main scripts
`Q1_Physical_modelling.m`, Q1.2-Q1.3, builds the coupled and decoupled state-space models and prints the matrices and eigenvalues.\
`Q1_4_drawing_shapes.m`, Q1.4, computes feedforward motor voltages for a given shape and simulates the print head path.\
`Q1_6_filament_cooling.m`, Q1.6, heat-transfer model of the printed filament.\
`Q3_1_filament_replacement.m`, Q3.1, hybrid simulation of printing the ellipse with filament replacement cycles.\
`Q3_2_draw_square.m`, Q3.2, hybrid simulation of printing a 40x40 mm square with guards for the corner switching.

## Functions
`parameters.m`, physical parameters of the 3D printer and hybrid controller parameters for Q3.\
`kinematics.m`, print head position and velocity from the motor angles.\
`dynamics.m`, coupled 6-state model of motors and print head.\
`dynamics_decoupled.m`, decoupled x and y dynamics.\
`nozzle.m`, nozzle model (temperature, capacitor voltage, inductor current).\
`define_shapes.m`, symbolic definitions for the shapes to print (ellipse, spade and circle).\
`print_trajectory.m`, determines feedforward voltages and initial conditions for a shape based on the dynamics.\
`simulate_trajectory.m`, simulates the trajectory using the decoupled x and y dynamics.\
`run_shape.m`, runs print_trajectory and simulate_trajectory for the chosen shape.\
`simulate.m`, hybrid system simulator with guards, reset maps and inputs per mode.
