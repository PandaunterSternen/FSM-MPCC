# Safe Lane Change by Using Finite State Machine and Control Barrier Functions

## Abstract
This project develops a decision-making system for autonomous vehicle lane changes using model predictive contouring control (MPCC). We combine a finite state machine (FSM) for obstacle detection with B-spline optimized trajectories. The system also integrates MPCC with control barrier functions (CBF) for safe path planning. Our approach is validated through simulations, demonstrating effective lane change maneuvers under various conditions.

## Features
- **High-level Decision-making Using FSM for Active Obstacle Avoidance**: [FSM.py](/MPCCCBFFSM/high_level_control/FSM.py)
- **B-spline Based Trajectory Optimization for Smooth Lane Changing**: [B_Spline_Curve.py](/MPCCCBFFSM/high_level_control/B_Spline_Curve.py)
- **Integration of MPCC and CBF for Safe Local Path Planning and Control**: [mpcc_optimization_function.py](/MPCCCBFFSM/MPCC_set/mpcc_optimazation_fuction.py)
- **Effective Handling of Various Road Conditions**: [sim_parameters.py](/MPCCCBFFSM/env/sim_parameters.py)
- **Proven Effectiveness Through Simulation**: Detailed in the accompanying thesis.

## Installation
To set up the project, follow these steps:
1. **Clone the repository**:
git clone https://github.com/PandaunterSternen/FSM-MPCC.git
2. **Navigate to the directory**:
cd /your_path/MPCCCBFFSM
3. **Install required packages**:
pip install -r requirements.txt

## Usage
To run the program, execute the following script:
/MPCC/MPCCCBFFSM/MPCC_CasADI_ms_avoid_withobs_noF_hL_c2.0_curveline_CBF++.py

### Dashboard
![Dashboard Image](/MPCCCBFFSM/images_and_video/Picture.png)

### Demo Video
![Simulation GIF](/MPCCCBFFSM/images_and_video/FSM_MPCC_speed_4X.gif)

## Tools Used
This project utilizes CasADi, an open-source tool for nonlinear optimization and algorithmic differentiation. CasADi is used for [solver]. More information about CasADi can be found on their [official website](https://web.casadi.org/) or [GitHub repository](https://github.com/casadi/casadi). CasADi is licensed under the LGPL.

## Acknowledgments
This project has benefited from several external resources and examples, particularly for the usage of the CasADi framework. We would like to thank:
- [CasADi_MPC_MHE_Python](https://github.com/tomcattiger1230/CasADi_MPC_MHE_Python.git) for CasADi application examples.
- [MPCC](https://github.com/alexliniger/MPCC.git) [Nonlinear_MPCC_for_autonomous_racing](https://github.com/nirajbasnet/Nonlinear_MPCC_for_autonomous_racing.git) for the mpcc framework.

We appreciate the contributions of the wider open-source community in supporting this project.

## Contributing
The framework is still relatively simple. We welcome contributions to improve this project. You can contribute in the following ways:
- Modifying and improving the existing code.
- Adding new features or enhancements.
- Optimizing performance and efficiency.

To contribute, please follow these steps:
1. Fork the repository.
2. Create a new branch for your feature.
3. Commit your changes.
4. Push to your branch and open a pull request.

## Contact
If you have any questions or need more information, please contact.




