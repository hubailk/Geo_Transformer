# Geo Transformer ROS2 Workspace

This workspace contains ROS2 packages for geographic coordinate transformations.

## Packages

### [geo_transformer](src/geo_transformer/README.md)

A ROS2 package that provides services for transforming between geographic coordinates (latitude, longitude, altitude) and local Cartesian coordinates (x, y, z) using GeographicLib.

## Quick Start

1. Install dependencies:
```bash
# Install GeographicLib
sudo apt-get install libgeographic-dev

# Install Python dependencies
pip3 install -r src/geo_transformer/requirements.txt
```

2. Build the workspace:
```bash
colcon build
```

3. Source the workspace:
```bash
source install/setup.bash
```

4. Run the transformer:
```bash
ros2 launch geo_transformer geo_transformer.launch.py
```

## Directory Structure

```
.
├── src/
│   └── geo_transformer/          # Main coordinate transformation package
├── build/                        # Build artifacts
├── install/                      # Install space
└── log/                         # Build and runtime logs
```

## Development

For detailed instructions on using and developing the packages, please refer to their individual README files in their respective directories.

## License

Apache 2.0 - See LICENSE file in each package for details.

## Contributing

Contributions are welcome! Please read the contributing guidelines in each package's documentation before submitting pull requests.
