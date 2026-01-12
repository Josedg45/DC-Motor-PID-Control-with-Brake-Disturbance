# PID-Controlled Plant with Functional State Classification

## Overview
This project presents the design, implementation, and validation of a mechatronic system controlled by a discrete PID controller and supervised through functional state classification using Artificial Intelligence techniques.

A physical plant is subjected to external disturbances and operational variations while maintaining reference tracking through PID control. The system behavior is analyzed to identify multiple functional states, which are then classified using both concrete and fuzzy classifiers.

This project was developed as part of a Mechatronics Engineering course focused on control systems and intelligent classification.

## Objectives
- Design and implement a discrete PID controller for a physical plant.
- Ensure stable reference tracking under different operating conditions and disturbances.
- Identify and characterize multiple functional states of the system.
- Develop and compare concrete and fuzzy classifiers for state detection.
- Integrate control, data acquisition, and classification into a single system.

## System Description
The system consists of:
- A physical mechatronic plant (e.g., motor, thermal system, level system).
- Sensors and actuators for real-time control and monitoring.
- A discrete PID controller responsible for regulation.
- External disturbances applied to generate different operating states.
- AI-based classifiers for functional state recognition.

## Methodology
1. **Plant Modeling**
   - Definition of inputs, outputs, and disturbances.
   - Mathematical modeling using transfer functions.

2. **PID Control Implementation**
   - Discrete PID controller design and tuning.
   - Validation of system response (overshoot, steady-state error, disturbance rejection).

3. **Functional State Definition**
   - Data acquisition under multiple operating scenarios.
   - Multivariable analysis to identify at least five functional states.

4. **Classifier Design**
   - Concrete classifier (e.g., K-Means).
   - Fuzzy classifier (e.g., Fuzzy C-Means).
   - Performance comparison between both approaches.

5. **Validation**
   - Real-time testing with controlled disturbances.
   - Evaluation of classification accuracy and system robustness.

## Tools and Technologies
- MATLAB / LabVIEW
- PID Control
- Signal Processing
- Machine Learning (Clustering)
- Fuzzy Logic
- Data Analysis and Visualization

## Results
The implemented PID controller ensures reference tracking with minimal steady-state error and acceptable overshoot.  
The classifiers successfully detect and differentiate the functional states of the plant, allowing real-time system monitoring and analysis.

## Conclusion
This project demonstrates the integration of classical control techniques with intelligent classification methods, highlighting the potential of AI tools for monitoring and diagnosis in mechatronic systems.

## Author
**[Felipe Mercado & José David Gómez Bedoya]**  
Mechatronics Engineer

## License
This project is intended for academic and educational purposes.
