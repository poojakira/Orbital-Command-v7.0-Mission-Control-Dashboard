# Orbital-Command-v3-Mission-Control-Dashboard

The software package Orbital Command v7.0 enables operators to control satellite missions while optimizing spacecraft flight paths. The system uses real orbital mechanics and stochastic hardware simulation and genetic algorithms to create a flight-ready environment which enables mission planning and GNC (Guidance Navigation and Control) operations

**Core Features**

i.  The Command Center system provides live global fleet tracking through actual TLE (Two-Line Element) data which it obtains from the spacetrack_full_catalog.3le.txt source. 

ii. The Flight Dynamics (GNC) system uses a Proximity Operations simulator which applies an Extended Kalman Filter (EKF) to process noisy sensor data for docking control. 

iii. The Mission Planning & Optimization system operates through a Genetic Algorithm (GA) optimizer which enables users to discover optimal orbits through Delta-V cost assessment and radiation risk evaluation and satellite shell (Starlink) collision risk assessment. 

iv. The certification process uses Automated Monte Carlo testing to confirm system performance by testing its limits under extreme situations. 

v. The Entropy Engine creates a simulation environment which applies IMU drift and thermal noise and radiation damage to the flight simulation. 

vi. The 3D Visualization system enables users to interact with 3D spacecraft models while displaying tactical flight paths through Plotly-based technology.

**Project Structure**

The structure of **Orbital Command v7.0** architecture divides its components into separate modules which include the user interface and physics simulation and autonomous control and system validation processes.

* **User Interface & Visualization:** The entry point is `app_dashboard.py` which serves as a Streamlit-based command center while `graphics_engine.py` and `model_3d.py` handle the interactive 3D rendering of the spacecraft and its tactical flight paths.
  
* **Core Physics & Data Ingestion:** The mission engine uses `mission_engine.py` to define orbital mechanics and environmental constants which work together with `data_processor.py` to extract and handle satellite data from `spacetrack_full_catalog.3le.txt`.
  
* **Guidance, Navigation, & Control (GNC):** The autonomous satellite "brain" operates through `rl_pilot.py` which uses the `gnc_kalman.py` module to determine satellite state through state estimation and filtering for precise satellite movements.
  
* **Optimization & Planning:** The mission strategy is handled by `ga_optimizer.py` which uses a genetic algorithm to determine orbital paths that offer maximum efficiency while preventing collisions and fuel consumption.
  
* **Reality Simulation & Hardware:** The `entropy_engine.py` program simulates sensor noise and hardware degradation while `subsystem_manager.py` monitors satellite health through its power usage and thermal control system.
  
* **Analysis & Deployment:** System reliability testing uses Monte Carlo methods in `system_analytics.py` to verify system reliability while the project becomes portable through Dockerfile and requirements.txt configuration.


**Installation & Setup**

**Requirements**

The system requires Python 3.9+ and the following libraries:

streamlit, plotly, pandas, numpy, scipy

skyfield (for TLE processing)

deap (for genetic optimization)

Running with Docker
You can containerize the environment using the provided Dockerfile:

                                                                     docker build -t orbital-command .
                                                                     
                                                                      docker run -p 8501:8501 orbital-command

 **Manual Setup**

i. Install dependencies:

                        pip install -r requirements.txt

ii. Launch the dashboard:

    
                         streamlit run app_dashboard.py

**Mission Phases**

i. The Command Center is responsible for monitoring satellite health and link budget status during all AOS and LOS periods which includes their global distribution tracking. 

ii. The Flight Dynamics team will perform simulated proximity operations together with docking maneuvers as part of their testing activities. 

iii, The Certification process requires pilots to complete Monte Carlo trials which will determine their ability to achieve required accuracy standards. 

iv. AI technology helps mission planners discover the safest altitude which requires the least amount of fuel for operation.


![dashboard 6](https://github.com/user-attachments/assets/177b5513-88e9-4822-8015-dc77089f9009)

          
