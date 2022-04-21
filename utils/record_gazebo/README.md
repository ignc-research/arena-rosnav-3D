Node for recording the gazebo world.

Different Camera Setups can be defined in `/resource/camera_setup.json` and selected by passing the name as a ros param. In the setup file each camera and their position and orientation is defined as well as the cameras that should be used as an array.

The cameras will start recording when receiving the /cmd/vel message for the first time. This should prevent to record useless stuff.

With the `src/create_video.py` script a video can be created out of the saved images. Therefore, you should pass the name of the camera as an argument.
