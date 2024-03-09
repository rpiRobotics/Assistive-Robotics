# Device Management Scripts

## Overview
``device_management_scripts`` directory contains scripts for managing a network of devices, particularly focusing on tasks like initial setup, git pulling, rebooting, cleaning ROS logs, shutting down, and managing ROS services on Dingo-based robots.

## Scripts Description

### General Device Management Scripts
- `all_devices_git_pull.bash`: Pulls the latest git repository changes onto all devices.
- `all_devices_initial_setup.bash`: Performs the initial setup for all devices.
- `all_devices_reboot.bash`: Reboots all devices.
- `all_devices_rosclean.bash`: Cleans ROS logs on all devices.
- `all_devices_shutdown.bash`: Shuts down all devices.
- `all_devices_terminal.bash`: Opens a new terminal window with tabs for SSH into each device.

### Dingo Robot Specific Scripts
- `all_dingo_bases_restart_ros_service.bash`: Restarts ROS services on all Dingo bases.
- `all_dingo_bases_update_ros_service.bash`: Updates ROS service files on all Dingo bases.

### Configuration Scripts
- Configuration scripts like `dingo_bases.sh`, `dingo_helpers.sh`, etc., define settings for different devices.

### Git Pull Scripts
- Specific scripts for performing git pull operations on different types of devices.

### Initial Setup Scripts
- Scripts for performing initial setup on various devices.

## Usage
To use these scripts:
1. Navigate to the `device_management_scripts` directory.
2. Run the desired script. For example:
   ```bash
   ./all_devices_reboot.bash
   ```


## Notes
- Customize the configuration scripts in the computers directory as per your network setup.
- The scripts are designed for password-based SSH authentication.