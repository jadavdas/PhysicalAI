## Physical AI

Physical AI refers to artificial intelligence systems that perceive, reason about, and act within the physical world. When applied to Autonomous Mobile Robots (AMRs), Physical AI marks a massive evolution from rigid, pre-programmed automation to true behavioral autonomy.

Instead of just processing digital text or code, Physical AI understands the physical laws of nature—such as gravity, friction, geometry, and spatial relationships—enabling mobile robots to operate safely and fluidly in unstructured, ever-changing environments like busy warehouses, factories, and hospitals.

Physical AI drives an ongoing Sense → Reason → Act feedback loop directly on the robot:

1. Sense (Perception): The AMR utilizes a multimodal sensor suite—including LiDAR, 3D cameras, and Inertial Measurement Units (IMUs)—to build a continuous, real-time understanding of its geometry and surroundings.

2. Reason (Cognition): Advanced Vision-Language-Action (VLA) models and world foundation models process this sensory data. The robot predicts how a scene will change if it moves forward, factoring in spatial context and dynamic obstacles.

3. Act (Execution): The AI translates its decisions into precise motor and actuator commands, altering speed, torque, and direction seamlessly to execute its task safely

In this workflow, let's first integrate a single sensor at a time and test it individually before putting them all together in the AMR to observe fully integrated autonomous behavior.

### [LIDAR](https://en.wikipedia.org/wiki/Lidar)

LiDAR (Light Detection and Ranging) is an active remote sensing technology that measures precise distances and generates high-resolution 3D representations of environments using pulsed laser light. By calculating the Time-of-Flight (ToF) for thousands of emitted light pulses to bounce off objects and return to the sensor, the system accurately maps the physical characteristics of a scene. In this video, red rays describes objects are detected and green rays describes free spaces.

https://github.com/user-attachments/assets/67c2a4da-fc92-45d2-87f2-f2227d791e76

### [CAMERA ](https://en.wikipedia.org/wiki/Vision-guided_robot_systems)

Cameras are optical sensors that provide mobile robots with rich geometric and photometric information required to perceive, interpret, and navigate their surroundings. Unlike active sensors (such as LiDAR), standard cameras capture ambient light data, translating raw pixel arrays into high-density color intensities and feature orientations that computer vision pipelines use to make autonomous decisions. The front camera acts as the primary visual navigator for the road ahead, while the bottom (underbody) camera functions as a specialized safety and maintenance scanner looking directly at the terrain and chassis beneath the vehicle.

https://github.com/user-attachments/assets/3d85a956-3882-47dd-ac61-423a84583f66

### [ULTRASONIC SENSOR](https://en.wikipedia.org/wiki/Ultrasonic_transducer)

An ultrasonic sensor is an active electronic transducer used in Autonomous Mobile Robots (AMRs) for short-range proximity detection and obstacle avoidance.

https://github.com/user-attachments/assets/d13b13dc-7cfb-49e7-9832-e2cbd8fa153f

https://github.com/user-attachments/assets/a6d29ccd-ec93-4e24-8dd5-6d631aa9d073

https://github.com/user-attachments/assets/45223c1f-0c1d-481d-9144-e5f3a63c6d88
