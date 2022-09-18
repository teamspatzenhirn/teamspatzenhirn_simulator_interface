# Team Spatzenhirn Simulator ROS Interface

This package contains ROS2 nodes for translating between the shared-memory interface of the simulator
and ROS messages.

## Usage

For our use-case, we have variants of our usual launchfiles that exchange the hardware interface with
the simulator interface nodes in this package.
The topics named directly correspond to the topics used during real-world driving for us.
If this is not the case for you, remapping the names probably works.

### SimulatorSpatzNode

Publishes simulated vehicle state and simulation status.
The simulation time requires special handling, see the section on time below!

#### Output Topics

| name    | type                                                                                                                | description                                 |
|---------|---------------------------------------------------------------------------------------------------------------------|---------------------------------------------|
| /clock  | rosgraph_msgs::msg::Clock                                                                                           | Current simulation time (see [time](#Time)) |
| /spatz  | [spatz_interfaces::msg::Spatz](https://github.com/teamspatzenhirn/spatz_interfaces/blob/master/msg/Spatz/Spatz.msg) | Simulated vehicle state                     |
| /rcmode | [spatz_interfaces::msg::RCMode](https://github.com/teamspatzenhirn/spatz_interfaces/blob/master/msg/RCMode.msg)     | Manual control override (set if paused)     |

### SimulatorDepthNode

Publishes the combined pointcloud of two Intel D435 cameras, as well as a decimated version of the same pointcloud.

#### Output Topics

| name                              | type                          | description                               |
|-----------------------------------|-------------------------------|-------------------------------------------|
| /depth/combinedPointcloud         | sensor_msgs::msg::PointCloud2 | Combined pointcloud of both depth cameras |
| /depth/combinedFilteredPointcloud | sensor_msgs::msg::PointCloud2 | 1/4 decimated pointcloud                  |

### SimulatorHardwareOutNode

Forwards controller outputs to the simulator.

#### Input Topics

| name              | type                                                                                                                              | description                           |
|-------------------|-----------------------------------------------------------------------------------------------------------------------------------|---------------------------------------|
| /control_setpoint | [spatz_interfaces::msg::ControlSetpoint](https://github.com/teamspatzenhirn/spatz_interfaces/blob/master/msg/ControlSetpoint.msg) | Vehicle steering and velocity command |

### SimulatorImageNode

Publishes the raw camera image in bayer format, as well as camera calibration data.
This node additionally publishes the TF2 transform from the vehicle frame to camera frame (`spatz` to `camera`).

#### Output Topics

| name                    | type                                                                                                                            | description                             |
|-------------------------|---------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------|
| /camera/image_raw       | sensor_msgs::msg::Image                                                                                                         | Main camera image, bayer encoded        |
| /camera/camera_info     | sensor_msgs::msg::CameraInfo                                                                                                    | CameraInfo corresponding to `image_raw` |
| /camera/extrinsic_calib | [spatz_interfaces::msg::ExtrinsicCalib](https://github.com/teamspatzenhirn/spatz_interfaces/blob/master/msg/ExtrinsicCalib.msg) | See message definition                  |

### SimulatorVisualOutNode

This nodes sends the currently planned trajectory to the simulator for visualization.
This node is currently not functional, since a dependency on other parts of our codebase
still needs to be resolved.

#### Input Topics

| name        | type                                                                                                                    | description                                                                                                                                                     |
|-------------|-------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| /trajectory | [spatz_interfaces::msg::Trajectory](https://github.com/teamspatzenhirn/spatz_interfaces/blob/master/msg/Trajectory.msg) | Current trajectory (TODO: replace with [ILQRTrajectory](https://github.com/teamspatzenhirn/spatz_interfaces/blob/master/msg/ILQRTrajectory/ILQRTrajectory.msg)) |

## Time

Since the simulator (usually) runs slower than real-time, and allows pausing and restarting,
the simulator is the ROS time source during simulation.
The `SimulatorSpatzNode` publishes the current simulation time on the `/clock` topic,
for other nodes to use.
Note that nodes that wish to use the current time need to be started with the `use_sim_time`
parameter set to `true`, in order to respect the `/clock` topic!

In a launch file:

```python
Node(
    package='teamspatzenhirn',
    executable='LaneDetectionNode',
    name='lane',
    parameters=[{'use_sim_time': True}],
)
```

In the console, for example with RVIZ:

```console
foo@bar:~$ rviz2 --ros-args -p use_sim_time:=true
```

## Status

The provided nodes (with the exception of SimulatorVisualOutNode) should be functional.
This is equivalent to what we use internally, although we use a copy of this code in our monorepo
which does not contain some of the included utility functions (which originally come from
other/shared libraries).

It has been our goal to integrate ROS interface directly into the simulator, but the existence
of these nodes currently makes this low priority for us. 

## Contributing

Contributions are welcomed and encouraged, especially in making this interface less dependent on
our custom ROS messages, and more in line with ROS standards (the simulator has been in use for
multiple years already when we switched to ROS).

If you changed something to make this work with your own software stack, let us know, and submit
a pull request!
