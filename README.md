# Agricultural-Robot
Designed the system architecture with three stacked subsystems:
– System Integration Subsystem: Built the architecture based on a stacking principle, developed lower-level
control with PID steering and odometry from encoders (programmed on Arduino), and communicated
modules via ROS. (Main Responsibility)(Video demo)
– Localization and Navigation Subsystem: Used Extended Kalman Filter to combine relative position data
from encoders and absolute position data from ArUco marker landmarks.(Video demo)
– Manipulator Arm Picking Subsystem.
