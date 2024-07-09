# Motion Capture to Crazyflie Control System

## Overview
This project integrates real-time motion capture data to control a Crazyflie drone's trajectory via a high-level command interface. The software is designed to support various motion capture systems by using the motioncapture library, making it versatile for different setups.

## Features
- **Motion Capture Integration:** Supports major motion capture systems such as Vicon, OptiTrack, Qualisys, and others.
- **High-Level Drone Commands:** Uses the high-level commander of the Crazyflie to execute complex flight patterns.
- **Flexible Trajectory Input:** Includes tools to generate and upload trajectories to the Crazyflie, allowing for dynamic flight experiences.

## Getting Started

### Prerequisites
- Crazyflie 2.0 drone.
- A supported motion capture system.
- Crazyflie Python API setup on the control system.

### Installation
1. Clone this repository to your local system.
2. Ensure that `cflib` and `motioncapture` libraries are installed:
   ```bash
   pip install cflib
   pip install motioncapture
