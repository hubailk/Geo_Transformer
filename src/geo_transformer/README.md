# Geo Transformer

A ROS2 package that provides services for transforming between geographic coordinates (latitude, longitude, altitude) and local Cartesian coordinates (x, y, z) using GeographicLib.

## Description

This package provides ROS2 services to:
- Set up a local coordinate system with a specified origin (latitude, longitude, altitude)
- Transform geographic coordinates to local Cartesian coordinates relative to the set origin

The coordinate transformations are handled by the GeographicLib library, which provides accurate geodetic calculations.

## Dependencies

- ROS2 Humble
- GeographicLib
- rclcpp
- std_msgs
- geographic_msgs

## Installation

1. Install GeographicLib:
```bash
sudo apt-get install libgeographic-dev
```

2. Clone this repository into your ROS2 workspace's src directory:
```bash
cd ~/your_workspace/src
git clone https://github.com/hubailk/Geo_Transformer.git
```

3. Build the package:
```bash
cd ~/your_workspace
colcon build --packages-select geo_transformer
```

## Usage

1. Source your ROS2 workspace:
```bash
source ~/your_workspace/install/setup.bash
```

2. Start the transformer node:
```bash
ros2 run geo_transformer geo_transformer_node
```

3. Use the provided services:

### Services

#### /local_coordinate_set
Sets the origin for the local coordinate system.

Input:
- `latitude` (float64): Latitude in degrees
- `longitude` (float64): Longitude in degrees
- `altitude` (float64): Altitude in meters

Output:
- `success` (bool): Whether the operation was successful
- `message` (string): Status message or error description

Example using command line:
```bash
ros2 service call /local_coordinate_set geo_transformer/srv/LocalCoordinateSet "{latitude: 47.6205, longitude: -122.3493, altitude: 158.5}"
```

#### /from_ll
Transforms geographic coordinates to local Cartesian coordinates.

Input:
- `latitude` (float64): Latitude in degrees
- `longitude` (float64): Longitude in degrees
- `altitude` (float64): Altitude in meters

Output:
- `x` (float64): X coordinate in meters
- `y` (float64): Y coordinate in meters
- `z` (float64): Z coordinate in meters

Example using command line:
```bash
ros2 service call /from_ll geo_transformer/srv/FromLL "{latitude: 47.6097, longitude: -122.3422, altitude: 27.0}"
```

## Testing

### Using the Launch File

The easiest way to test the package is using the provided launch file, which starts both the transformer node and test client:

```bash
ros2 launch geo_transformer geo_transformer.launch.py
```

### Manual Testing

You can also run the nodes separately:

1. Start the transformer node:
```bash
ros2 run geo_transformer geo_transformer_node
```

2. In another terminal, run the test client:
```bash
ros2 run geo_transformer test_client
```

The test client:
1. Sets the local coordinate system origin at the Space Needle location
2. Converts the Pike Place Market coordinates to local XYZ coordinates relative to the Space Needle

## License

Apache 2.0 - See LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
