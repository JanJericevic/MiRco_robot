# Docker convenience scripts

Docker convenience scripts for a more optimal workflow.  
Copy them to you `~/.local/bin` and create aliases in your `.bashrc`.

- **`container_prompt.sh:`** opens another interactive bash prompt of a running container. Currently only works for one running container.
- **`volume-ros-docker.sh:`** start a container with:
    - enabled NVIDIA GPU
    - enabled use of GUI aps through X server
    - using volumes to share folders
    - exposed input devices - used for joystick teleop