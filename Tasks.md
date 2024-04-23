## Task Description

Control the Turtlebot using ROS with a USB Foot Pedal interface. (max 4 team members, 1 team)
- This project involves using the Raspberry Pi platform.
- A small rover chassis should be controlled, to drive a circuit in a 2m X 2m space. The speed and direction of the rover should be controlled by the Foot Pedal. The Foot Pedal should be connected via USB OTG to the nucleo board, which should then broadcast the commands via ROS.
- The rover should also be tracked with a motion model and the onboard LiDar, with the display on either using a web dashboard

## Tasks

- Web Dashboard (Rachael)
    - Determine what the user interface has to display, including a map showing the location of the turtle-bot, the current robot state, the current control state.
    - Milestone
        - System Overview
        - Partial code implementation.
- TurtleBot Localisation (Lauchie)
    - Determine how to get the LiDAR data.
    - Determine how to localise the turtlebot in space.
    - Kalmann filter
    - Milestone
        - DIKW Pyramid abstraction.
        - Partial code implementation.
- TurtleBot Integration (Gabe)
    - Xbox controller for 
    - Integrating MQTT and ROS.
    - Handling ROS configuration.
    - Milestone:
        - Has 5 KPIs which are sufficiently described in detail.
        - System sensor integration.
        - Partial code implementation.
- Foot pedal (James)
    - Firmware for the nucleo board to interface with the foot pedal.
    - Getting the foot pedal.
    - Milestone
        - Network protocol diagram
        - Partial code implementation.
