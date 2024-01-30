# Safe Lane Change by Using Finite State Machine and Control Barrier Functions

## Abstract
For autonomous vehicles, performing lane change maneuvers in a safe and smooth manner is crucial. This project develops a high-level decision-maker for model predictive contouring control (MPCC). By integrating a finite state machine (FSM) with a safety judgment mechanism, our approach actively avoids obstacles and optimizes lane changing trajectory using B-splines. In the lower execution layer, MPCC is combined with the control barrier function (CBF) for safe local path planning and optimal control input determination under a wide range of road conditions. Simulation results demonstrate the effectiveness of our method.

## Features
- **High-level Decision-making Using FSM for Active Obstacle Avoidance**: Implemented in [FSM.py](/MPCCCBFFSM/high_level_control/FSM.py).
- **B-spline Based Trajectory Optimization for Smooth Lane Changing**: See [B_Spline_Curve.py](/MPCCCBFFSM/high_level_control/B_Spline_Curve.py) for details.
- **Integration of MPCC and CBF for Safe Local Path Planning and Control**: Core functionality located in [mpcc_optimazation_fuction.py](/MPCCCBFFSM/MPCC_set/mpcc_optimazation_fuction.py).
- **Effective Handling of Various Road Conditions**: Adaptations for different conditions are handled in [sim_parameters.py](/MPCCCBFFSM/env/sim_parameters.py).
- **Proven Effectiveness Through Simulation Results**: Simulation scripts and results can be found in the thesis of the same name.


## Installation
To set up the project, follow these steps:
1. Clone the repository:
git clone https://github.com/PandaunterSternen/FSM-MPCC.git
2. Navigate to the `MPCCCBFFSM` directory:
cd /your_path/MPCCCBFFSM
3. Install the required packages:
pip install -r requirements.txt

## Usage
To run the program, execute the following script:
/MPCC/MPCCCBFFSM/MPCC_CasADI_ms_avoid_withobs_noF_hL_c2.0_curveline_CBF++.py

### Dashboard
A dashboard for real-time monitoring is provided :
![Example Image](/MPCCCBFFSM/images_and_video/Picture.png)

### Demo Video
Watch the lane changing in action 
![Alt Text](/MPCCCBFFSM/images_and_video/FSM_MPCC_speed_4X.gif)

## Contributing
We welcome contributions to improve this project. You can contribute in the following ways:
- Modifying and improving the existing code.
- Adding new features or enhancements.
- Optimizing performance and efficiency.

To contribute, please follow these steps:
1. Fork the repository.
2. Create a new branch for your feature.
3. Commit your changes.
4. Push to your branch and open a pull request.

## License
This project is licensed under the [MIT License](LICENSE link here) - see the LICENSE file for details.

## Contact
For any queries or further information, please contact [your email/contact information].




