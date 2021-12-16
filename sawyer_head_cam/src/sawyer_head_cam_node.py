  <node name="image_view" pkg="image_view" type="image_view" output="screen" >
  <remap from="/image" to="/usb_cam/image_raw"/>
  </node>

    <node type="rviz" name="rviz" pkg="rviz" args="-d $~.rviz/USBCAM_pose.rviz" />
