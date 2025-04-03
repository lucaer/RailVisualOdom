# RailVisualOdom
Visual Odometry framework for railway critic system in ETCS - CBTC

This work addresses the critical need to “smartify” maintenance vehicle through the development of a modular, plug-in visual odometry framework. 

This is done by the implementation of the perception layer and odometry layer, detailing the integration of object detection and filtering techniques for precise vehicle pose estimation.

Traditional railway signaling and train control systems, relying on trackside balises and onboard phonic wheels, present significant maintenance challenges, safety risks, and substantial lifecycle costs due to their inherent complexity and susceptibility to environmental wear.
This work proposes a novel approach that eliminates these legacy components, replacing them with a streamlined system for visual odometry utilizing fused information from inertial measurement units (IMUs), LiDAR and cameras with advanced computer vision techniques.
The results of this research demonstrates the feasibility and efficacy of a vision-based odometry approach, paving the way for the development of intelligent, adaptable, and cost effective solutions for the railway industry.


Railway systems, by their very nature, constitute critical infrastructure where failures can have severe repercussions for the overall public safety.
So, each technological improvement in this field and the use of newborn methodology need a long process of assessment before large scale implementation.
At the international level, the European Union Agency for Railways (ERA) establish common safety methods across European railway systems while and additional standards developed by CENELEC define detailed technical requirements for safety-critical systems.

<img width="878" alt="Screenshot 2025-04-04 alle 00 10 55" src="https://github.com/user-attachments/assets/87246aeb-414e-4d45-8ff1-a240390baccc" />

The field of application of this module include railway maintenance vehicle used to perform a variety of essential tasks, including track inspection and line maintenance.

Integrating these vehicles into modern train control systems, such as the European Train Control System (ETCS), presents a significant opportunity to enhance safety and operational efficiency.
However, many older maintenance vehicles lack the necessary onboard equipment for ETCS integration, necessitating innovative solutions to "smartify" them.
This perception kit with cameras, LiDAR, and IMUs, with small factor processing unit, allows for the development of retrofit systems enabling ETCS compatibility without requiring extensive modifications to the vehicle's infrastructure. 
Of course, this new systems with the integration of AI need to comply with the stringent safety requirements.

<img width="876" alt="Screenshot 2025-04-04 alle 00 15 10" src="https://github.com/user-attachments/assets/4e82833d-c7a1-4b21-8296-2d32b84d1f26" />

-----------------------------------------------------------------------------------------------
PERCEPTION LAYER
The object detection module use YOLOv11 in conjunction with the ByteTrack tracking algorithm, delivering as its primary output bounding box coordinates and unique object tracking identifiers, together achieving optimized computational efficiency for real-time performance.
Then to establish a spatial correspondence between image-based object detected and the 3D spatial data, precise distance measure are obtained with the LiDAR point-cloud considering, projected points that falls within the bounding box region. 
To enhance the quality of output, additional functions for outlier removal and clustering are used.

<img width="876" alt="Screenshot 2025-04-04 alle 00 17 36" src="https://github.com/user-attachments/assets/5c4e83a8-0319-4cb6-9559-94151b28716f" />

ODOMETRY LAYER

To represent the dynamics of the train a state vector is defined together with a transition matrix.
The update of the state of the train is done by a Kalman Filter assuming for simplicity a constant acceleration model.

<img width="887" alt="Screenshot 2025-04-04 alle 00 19 01" src="https://github.com/user-attachments/assets/4a1a0f6d-be58-4645-ac50-2cfc8698d98f" />

The system, effectively constrains the error accumulation characteristic of IMU-based navigation by incorporating the visual odometry data.
This approach intentionally excludes GPS, and balise correction following the previously established justifications. And make it possible to implement the AI function according to the regulations.

<img width="869" alt="Screenshot 2025-04-04 alle 00 26 23" src="https://github.com/user-attachments/assets/69078a01-c9ac-4884-ad18-35be3b8c0a00" />


