# SDL - cuTAMP

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/index.html)
[![Python](https://img.shields.io/badge/python-3.11-blue.svg)](https://docs.python.org/3/whatsnew/3.11.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)

## Overview

This project utilizes cuTAMP within Isaac Sim to perform **S**elf-**D**riving **L**aboratories


## Installation

### Requirements

- Set up IsaacSim ROS2 Environment
  - install [Isaac Sim ROS Workspace](https://github.com/isaac-sim/IsaacSim-ros_workspaces)

    ```bash
    cd ~/
    git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
    ```

    - Copy `tamp_interfaces` package and paste into `~/IsaacSim-ros_workspaces/humble_ws/src`

    - build
        ```bash
        cd ~/IsaacSim-ros_workspaces
        ./build_ros.sh -d humble -v 22.04
        ```

### Setup Workspace

```bash
sudo apt-get install build-essential
sudo apt-get install gcc-11 g++-11
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200
```

- install isaacsim

    ```bash
    cd ~/
    git clone https://github.com/isaac-sim/IsaacSim.git
    ```

    ```bash
    cd IsaacSim
    git lfs install
    git lfs pull
    ```

    ```bash
    ./build.sh
    ```


- create workspace

    ```bash
    cd ~/
    git clone git@github.com:chohh7391/sdl_cutamp_ws.git
    ```

    - create cutamp conda env

    ```bash
    conda create -n cutamp python=3.10 -y
    conda activate cutamp
    ```

    ```bash
    cd ~/sdl_cutamp_ws/src/cuTAMP
    pip install -e .
    ```

    ```bash
    sudo apt install git-lfs
    git lfs install
    ```

    ```bash
    cd curobo

    # This can take up to 20 minutes to install
    pip install -e . --no-build-isolation
    pip install lark
    ```

    ```bash
    # Optional: Verify that all unit tests pass
    pip install pytest
    python -m pytest .
    cd ..
    ```

    ```bash
    conda deactivate
    cd ~/sdl_cutamp_ws
    colcon build
    ```


# Demo

## Isaacsim with ROS2 Launch

```bash
source /opt/ros/humble/setup.bash
source ~/sdl_cutamp_ws/install/local_setup.bash
```

```bash
ros2 launch isaacsim run_isaacsim.launch.py standalone:=$HOME/sdl_cutamp_ws/src/isaacsim/scripts/standalone/simulation.py install_path:=$HOME/IsaacSim/_build/linux-x86_64/release exclude_install_path:=home/home/sdl_cutamp_ws/install ros_installation_path:="/home/home/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local_setup.bash,/home/home/IsaacSim-ros_workspaces/build_ws/humble/isaac_sim_ros_ws/install/local_setup.bash"
```

## TAMP


- Run TAMP Server

    ```bash
    source /opt/ros/humble/setup.bash
    source ~/sdl_cutamp_ws/install/local_setup.bash
    conda activate cutamp
    export SYSTEM_LIBSTDCXX_PATH="/usr/lib/x86_64-linux-gnu/libstdc++.so.6"
    LD_PRELOAD="${SYSTEM_LIBSTDCXX_PATH}" ros2 run tamp tamp_server.py
    ```

- Run TAMP Client

    - run client

    ```bash
    source /opt/ros/humble/setup.bash
    source ~/sdl_cutamp_ws/install/local_setup.bash
    ros2 run tamp tamp_client.py
    ```    
    
    - set tamp env

      - pouring
        ```bash
        (csuite) set_tamp_env pouring
        ```

      - stirring
        ```bash
        (csuite) set_tamp_env stirring
        ```

    - plan
    ```bash
    (csuite) plan
    ```

    - execute
    ```bash
    (csuite) execute
    ```

    - change tool
      - ag95
        ```bash
        (csuite) change_tool ag95
        ```

      - 2f_85
        ```bash
        (csuite) change_tool 2f_85
        ```





## Sim Control

reference: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_simulation_control.html#enable-manually

```bash
sudo apt install ros-humble-simulation-interfaces
```

- how to get robot data

    - Does Entity exiting?
    ```bash
    ros2 service call /get_entity_info simulation_interfaces/srv/GetEntityInfo "{entity: '/World/Robot'}"
    ```

    - Get entities with paths starting with 'World'
    ```bash
    ros2 service call /get_entities simulation_interfaces/srv/GetEntities "{filters: {filter: '^/World'}}"
    ```

    - Get States like (pose, twist, acceleration)
    ```bash
    ros2 service call /get_entity_state simulation_interfaces/srv/GetEntityState "{entity: '/World/Robot'}"
    ```

- how to delete or spawn entity
  
    - Delte Entity
    ```bash
    ros2 service call /delete_entity simulation_interfaces/srv/DeleteEntity "{entity: '/World/Robot'}"
    ```

    - Spawn Entity
    ```bash
    ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'PositionedEntity', allow_renaming: false, uri: '/path/to/model.usd', initial_pose: {pose: {position: {x: 0.0, 0.0, 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}"
    ```

- how to reset simulation

    - Reset Simulation
    ```bash
    ros2 service call /reset_simulation simulation_interfaces/srv/ResetSimulation
    ```

- how to set entity state

    - Set only position and orientation
    ```bash
    ros2 service call /set_entity_state simulation_interfaces/srv/SetEntityState "{
        entity: '/World/Cube',
        state: {
            header: {frame_id: 'world'},
            pose: {
            position: {x: 1.0, y: 2.0, z: 3.0},
            orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
            },
            twist: {
            linear: {x: 0.0, y: 0.0, z: 0.0},
            angular: {x: 0.0, y: 0.0, z: 0.0}
            }
        }
    }"
    ```

    - Set position, orientation and velocity (for entities with rigid body physics):
    ```bash
    ros2 service call /set_entity_state simulation_interfaces/srv/SetEntityState "{
        entity: '/World/Cube_1',
        state: {
            header: {frame_id: 'world'},
            pose: {
            position: {x: 1.0, y: 2.0, z: 3.0},
            orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
            },
            twist: {
            linear: {x: 0.5, y: 0.0, z: 0.0},
            angular: {x: 0.0, y: 0.0, z: 0.1}
            }
        }
    }"
    ```