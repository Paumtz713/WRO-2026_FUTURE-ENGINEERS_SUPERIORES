# Software

This folder contains the complete control software of the robot.

## Main features

- Ultrasonic-based navigation (threshold control)
- IMU-based yaw tracking for lap counting
- HuskyLens-based color detection (red/green pillars)
- Obstacle avoidance logic
- Parking sequence (in development)

## Notes

The robot does not currently use PID control. Steering decisions are based on distance thresholds and IMU correction.
