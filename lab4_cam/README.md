# lab4_cam

We created and edited these files:
  1. nominal.launch
  2. sawyer_wrist_track.launch
  3. both_sawyer_cams.py
  4. trans_webcam_to_base.py

The package name is undescriptive because we had intially used the lab 4 package, and once everything was made we didn't want to edit the name of the package and mess up the
structure of our ROS workspace.

This package handles the AR tracking on both the Sawyer head camera and the usb camera as well as broadcasting the resultant tf between the Sawyer base and the usb_cam to the
tf static tree. This effectively bridges the tf trees between the usb_cam and Sawyer and allows us to find the pose of the AR tag and feed it to the path planner even when the
Sawyer arm and Allegro hand are obstructing the head camera. This is because we move the AR tag to be in view of the head camera and usb camera and run the trans_webcam_to_base
node which will look for messages from the two AR tracking topics and do matrix multiplication to get the pose from the base to the usb camera. Thus we can only rely on the 
usb camera for the path planning as long as its view remains unobstructed.

Alex Chemali was the main contributer with Tristan Schwab editing assisting on nominal.launch and setup of the package.
