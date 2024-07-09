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
   ```

### Configuration
Set the URI of your Crazyflie and the hostname of your motion capture system in the script:

```bash
uri = 'radio://0/80/2M/E7E7E7E7E7'  # Update as needed
host_name = '192.168.0.50'  # Update with your mocap system's address
```

### Usage
To run the project:

Make sure the Crazyflie and the motion capture system are powered and operational.
Execute the script to start the flight sequence:

```
python your_script_name.py
```
## Contributing
Contributions to enhance this project are welcome. Please fork the repository and submit a pull request with your improvements.

## License
This project is licensed under the GNU General Public License v3.0 - see the LICENSE file for details.

## Acknowledgments
Bitcraze AB for the Crazyflie platform.
All motion capture system vendors supporting the development of open interfaces
