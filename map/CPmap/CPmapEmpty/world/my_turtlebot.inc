define kinect ranger
(
  sensor
  (
    range_max 6.5 # [m]
    fov 58.0
    samples 640 # readings the sensor take
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ] #how big the sensors are [m]
)

define turtlebot position
(
  pose [ 3.0 1.0 0.0 0.0 ]

  #odom_error [0.03 0.03 999999 999999 999999 0.02] #The amount of error that the robot will make in the odometry recordings.

 #size [ 1.0 1.0 1.8 ] #robot size [m]
  size [ 0.6 0.6 1.8 ]
  #origin [ 0.5 0.5 0.8 0.0 ]
  origin [ 0.3 0.3 0.8 0.0 ]
  gui_nose 0
  drive "omni" # Tells the odometry how the robot is driven:  "diff" or "car" or "omni"
  color "gray"
  
  #kinect(pose [ -0.1 0.0 -0.11 0.0 ]) # x, y, z, yaw

  kinect(pose [ 0.5 0.5 0.8 0.0 ]) # x, y, z, yaw

)
