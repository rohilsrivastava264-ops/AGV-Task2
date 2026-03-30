# AGV Simulation Project

This project implements a robot navigation system in a 2D environment using C++.

## Features
- Random waypoint navigation
- Obstacle avoidance using LiDAR raycasting
- Potential field-based motion planning
- Noise filtering using moving average
- Dynamic obstacles

## Concepts Used
- Geometry and vectors
- atan2 for angle computation
- Distance calculations
- Potential fields (attractive + repulsive forces)
- Smoothing filters

## How it works
The robot:
1. Moves toward randomly generated waypoints
2. Uses LiDAR to detect obstacles
3. Applies repulsive forces to avoid collisions
4. Adjusts speed based on surroundings

## Controls
- Autonomous mode (no keyboard input required)

## Author
Rohil Srivastava
