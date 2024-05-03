*`Established: 2024/05/03`* *`Updated: 2024/05/03`*

## About The Project
The QoS profile management service for the robot vehicle ver. 1 project.

The service is used to manage the topic QoS profile for all the devices in the vehicle, and send the update signal to the client device. The client device can request a new QoS profile from the server and update publisher or subscription QoS by itself.

**NOTE:** The implementation of communication between the QoS server and the client was implemented under `QoSUpdateNode` at `vehicle_interfaces/qos.h`. The client device can easily communicate with the server by inheriting the `QoSUpdateNode` class or the derived `VehicleServiceNode` class.

**NOTE:** To enable the funcitons of `QoSUpdateNode`, make sure to pass the correct service name to the `QoSUpdateNode` constructor. If user want to disable the functions, set the service name to empty string.

For the client device, there are several functions can be called:
- `addQoSTracking()`: Add topic name to the tracking list to receive the QoS update signal from the server.
- `addQoSCallbackFunc()`: Add callback function to receive the new QoS profile from the server.
- `requestQoS()`: Request the new QoS profile from the server.

If the `QoSUpdateNode` was enabled, the node will create a directory `qosDirPath` defined in the `service.json` file to store the QoS profile for each topic device (e.g. publisher or subscription). The QoS profile is stored in the `qosDirPath` directory with the file name as the topic name.

If calling `addQoSTracking()` function, the node will add topic name to list and check whether the QoS profile file exists in the `qosDirPath` directory. If the file exists, the node will read the QoS profile from the file and return the QoS profile to the topic device. Otherwise, the node will return the default QoS profile to the topic device.


## Getting Started

### Prerequisites
- ROS2 `Foxy` or later (`Humble` recommended)
    Install ROS2 from official website: [ROS2 official website](https://docs.ros.org/en/humble/Installation.html) or simply run the following command to automatically install ROS2:
    ```bash
    curl -fsSL ftp://61.220.23.239/scripts/install-ros2.sh | bash
    ```
    **NOTE:** The script only supports `Foxy` and `Humble` versions depending on the Ubuntu version.
    **NOTE:** The script will create a new workspace at `~/ros2_ws`.
    **NOTE:** The script will create an alias `humble` or `foxy` for global ROS2 environment setup (e.g. `source /opt/ros/<$ROS_DISTRO>/setup.bash`) depending on the ROS2 version.
- [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git)
- nlohmann-json3-dev

The required packages are listed in the `requirements_apt.txt` file. Install the required packages by running the following command:
```bash
xargs sudo apt install -y < requirements_apt.txt
```
**NOTE:** The required packages will be installed automatically while installing the package using the (`vcu-installer`)[https://github.com/cocobird231/RV1-vcu-install.git].


### Installation
There are two ways to install the package: manually or using the `vcu-installer`. 

#### Install Manually
1. Check if `vehicle_interfaces` package is installed. If not, install the package by following the instructions in the [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git).
2. Clone the repository under `~/ros2_ws/src` and rename it to `cpp_qosserver`:
    ```bash
    git clone https://github.com/cocobird231/RV1-qosserver.git cpp_qosserver
    ```
3. Change the directory to the `~/ros2_ws` workspace and build the package:
    ```bash
    # Change directory to workspace.
    cd ~/ros2_ws

    # Source the local environment.
    . install/setup.bash

    # Build the package.
    colcon build --symlink-install --packages-select cpp_qosserver
    ```
    **NOTE:** The package is installed in the local workspace.


#### Install Using `vcu-installer`
1. Run the installer and press `Scan` button under Package Management window. If the installer not installed, install the installer by following the instructions in the [`vcu-installer`](https://github.com/cocobird231/RV1-vcu-install.git).

2. Checked the `QoS Server` checkbox under package list, right-click to modify the internet setting, then press the `Install` button to install the package.

3. The installer will create the start-up script for the package under `/etc/xdg/autostart` directory. The package will be started automatically after the system boot-up.


## Usage
The package contains two executables: `server` and `control`. The `server` is the main service to manage the QoS profile for all the devices in the vehicle, and the `control` is a demonstration tool to communicate with the server.

### Run the Main Service
1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the main service:
    - Using the `launch`:
        ```bash
        ros2 launch cpp_qosserver launch.py
        ```
        **NOTE:** The launch file parsed the `common.yaml` file to set the parameters. The `common.yaml` file is located in the `cpp_qosserver/launch` directory.
        **NOTE:** The `common.yaml` file default the namespace to `V0`.

    - Using the `run`:
        ```bash
        ros2 run cpp_qosserver server
        ```

### Demonstration Tool
1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the control service:
    ```bash
    ros2 run cpp_qosserver control
    ```
    **NOTE:** If the main service is using the namespace (e.g. `V0`), the control service should use the same namespace to control the main service:
    ```bash
    ros2 run cpp_qosserver control --ros-args -r __ns:=/V0
    ```


## Description

### Communicate with the Service
For the derived class of `QoSUpdateNode`, the client device is used to communicate with the server using `QosReg.srv` and `QosReq.srv` which are defined in the `vehicle_interfaces/srv`. 

#### `QosReg.srv`
The service is used to register the QoS profile for the client device. The service contains the following fields:
```.srv
# Request field
## Set topic_name QoS profile into temporary qmap
string topic_name

## Set to 'publisher', 'subscription' or 'both'
string qos_type "both"

## Remove topic_name profile in temporary qmap. If true, server will ignore this QoS profile
bool remove_profile 0

## Clear temporary qmap. If true, server will ignore this QoS profile. 
bool clear_profiles 0

## Set ture if all QoS profile setting completed. If true, server will ignore this QoS profile
bool save_qmap 0

## QoS Profile Setting
QosProfile qos_profile

# Response field
## response is true if value accepted, otherwise server ignore the request and response false.
bool response

## If <response> response flase, describes the reason.
string reason

## Unique ID describes current qos profile. qid changed while save_qmap requests true.
uint64 qid
```

The `QosProfile` describes the QoS profile which contains the following fields:
```.msg
# Ref to enum rmw_qos_history_policy_t
int8 history

# Ref to size_t depth
int64 depth

# Ref to enum rmw_qos_reliability_policy_t
int8 reliability

# Ref to enum rmw_qos_durability_policy_t
int8 durability

# Ref to struct rmw_time_t deadline
float64 deadline_ms

# Ref to struct rmw_time_t lifespan
float64 lifespan_ms

# Ref to enum rmw_qos_liveliness_policy_t
int8 liveliness

# Ref to struct rmw_time_t liveliness_lease_duration
float64 liveliness_lease_duration_ms
```
**NOTE:** The current QoS server will only store the first four QoS profile settings (i.e. `history`, `depth`, `reliability`, and `durability`). The `deadline`, `lifespan`, `liveliness`, and `liveliness_lease_duration` will be ignored due to the compatibility issue.

**NOTE:** The `qid` is the unique ID for the entire QoS profiles. The `qid` will be changed when some of the QoS profiles are updated. The `qid` is used to identify the current QoS profile for the client device.

#### `QosReq.srv`
The service is used to request the QoS profile for the client device. The service contains the following fields:
```.srv
# Request field
string topic_name # Get topic_name QoS setting
string qos_type "both" # Set to 'publisher', 'subscription' or 'both'

# Response field
## response is true if value accepted, otherwise server ignore the request and response false
bool response

## If <response> response flase, describes the reason.
string reason

## Unique ID describes current qos profile
uint64 qid

## topic names
string[] topic_name_vec

## QoS types
string[] qos_type_vec

## QoS Profile Setting
QosProfile[] qos_profile_vec
```
**NOTE:** The size of `topic_name_vec`, `qos_type_vec`, and `qos_profile_vec` should be the same.

### Parameters Callback
The QoS server support the parameter callback for behavior control. The parameters include:
- `enabled_publish`: Enable to publish the update signal to the client device.
- `publish_interval_ms`: Set the publish interval in milliseconds.

**NOTE:** The QoS server is using `OnSetParametersCallbackHandle` to handle the parameter callback event. For the control side, use the client with `rcl_interfaces::srv::SetParametersAtomically` type to set the parameters.


### `common.yaml` File
For QoS server, the only parameter is used in common.yaml file is `recordFilePath`, which is used to set the directory path to store the QoS profile for each topic device.