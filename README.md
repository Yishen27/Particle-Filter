# Particle-Filter
----
This is a project aiming at budilding a 2 dimensional particle filter in C++ that can realize localization in an unknown environment. 
The simulated robot with the sensing code is initially set to an unknown location with a map of the environment and a noisy GPS estimate 
of its location. The code is able to localize the robot position and yaw with the sensing and controlling data from the robot at each
time step. 

## Key Files 
### Folder *src*
This is the folder contains all c++ files constitute the program.

- main.cpp
  Main program communicates with the Simulator receiving sensing data, calls functions to run the particle filter.
  
- particle_filter.cpp and particle_filter.h
  Codes for implementing the particle filter. *particle_filter.h* defines all relevent data structures and *particle_filter.cpp* defines
  all the functions.
  
- helper_functions.h
  Contains all helper functions.
  
### Project_readme.md
The original readme file for this project.
