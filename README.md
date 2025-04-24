# CARLA Bicycling Simulator Scripts

This repository contains custom modifications to the CARLA simulator's example folder. These scripts extend CARLA's functionality to enable gradient-enabled bicycling simulation.

## Table of Contents

- [Overview](#overview)
- [Setup](#setup)
- [License](#license)

## Overview

This project builds upon the example scripts provided with the CARLA simulator, incorporating modifications originally made by Paul Pabst and Patrick Malcolm ([GitHub Repository](https://github.com/patmalcolm91/CARLA-Bike-Simulator-Tools)) to adapt them for use with a bicycling simulator. Further custom scripts have been added to support integration with a Wahoo KICKR Bike via ANT+ interface. The system enables real-time data exchange for gradient, speed, cadence, and power, supporting gradient-responsive cycling simulation.

To use this project, please refer to the official CARLA Simulator documentation and ensure you install the appropriate version of both the CARLA simulator and its Python API. This implementation has been tested with CARLA version 0.9.15. It has not been tested with later CARLA versions based on Unreal Engine 5.x.

This repository is provided "as is," without warranty. The University of Wuppertal, the Chair of Bicycle Traffic, and the individual authors assume no liability for any issues arising from its use.

Note: AI tools (Claude, Mistral and Github Copilot) supported routine code generation and early drafts. Final implementations reflect design decisions, custom logic, and domain-specific adaptations made by the team.

## Setup

1. Clone/Download the repository.

2. Copy all files from the `src` folder to your CARLA installation's Python examples directory:
   ```
   $CARLA_PATH$/PythonAPI/examples
   ```
   **Important:** This will overwrite existing files in the target directory. Make backups of any original CARLA files you wish to preserve before proceeding.

3. Install the required Python dependencies:
   ```
   py -m pip install -r requirements.txt
   ```

4. Ensure the CARLA Simulator Python library is installed and accessible. For installation guidance, refer to the [CARLA Simulator documentation](https://carla.readthedocs.io/en/latest/start_quickstart/).

5. Set up ANT+ connectivity:
   - Verify that your ANT+ connection is operational on the simulation PC
   - Default connection port is `8080` (can be modified in `BikeSensor.py` if needed)
   - For device-specific implementation, follow the guidelines in the [Garmin ANT Wireless Networks documentation](https://developer.garmin.com/ant-program/overview/)
   - Ensure your ANT+ receiver is properly connected and recognized by the system

6. Launch the CARLA simulator using one of these methods:
   - Run the packaged executable version, OR
   - Play the simulation in the Unreal Editor if using a compiled build
   
   **Note:** The CARLA simulator must be running before executing any Python scripts in the following steps. Scripts will attempt to connect to the simulator on the default port.

7. Run the simulation script: 
   ```
   py manual_control_simulator.py -x SPAWN_POINT
   ```
   - Where `SPAWN_POINT` is the numerical ID of the desired spawn location for your ego bicycle. (Please refer to [CARLA documentation on map spawn points](https://carla.readthedocs.io/en/latest/tuto_G_getting_started/#using-and-visualizing-map-spawn-points))


**Note 1**: Sample Arduino code for hardware integration is provided in the `Arduino` folder (compatible with Arduino UNO Rev 3). This code handles the interface between speed and steering measuring sensors and the PC via an Ethernet connection.

**Note 2**: An example experiment log file is available in the `experiment_log` folder. These sample logs demonstrate the data structure, formatting, and typical metrics collected during simulation runs.

## License

This repository is licensed under the MIT License. See the [LICENSE](LICENSE.txt) file for details.

