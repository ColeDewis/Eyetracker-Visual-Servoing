# Simulator
A basic simulator for a Kinova Gen3 Arm. Attempts to roughly replicate some of the functionality of the kortex_driver module.

Currently:
- Displays camera feeds
- Publishes transformation frames of the robot arm, as well as camera positions
- Publishes a joint state similar to kinova

Tracking is automatically done within simulator for segmentation ids specified in the launch file. Otherwise you can run a tracking node in parallel.

## Resources
The simulator uses robosuite: https://robosuite.ai/docs/overview.html. The docs here are OK, but sometimes you likely will need to look through the source code here: https://github.com/ARISE-Initiative/robosuite

In order to handle robot kinematics, we use Peter Corke's Robotics Toolbox: https://github.com/petercorke/robotics-toolbox-python as well as the spatial math toolbox in small amounts https://github.com/bdaiinstitute/spatialmath-python/