Header      header            #Timestamp is the "measurement time" from the manual.
comms_msgs/Node source            # The transmitting node.

uint8       FIX_ANGLES=1
uint8       FIX_POSITION=2

uint8       position_fix      # Position_fix is an enum value for the type of measurement.

geometry_msgs/Vector3 fix123
geometry_msgs/Vector3 fix432
geometry_msgs/Vector3 fix341
geometry_msgs/Vector3 fix412
geometry_msgs/Vector3 fix153
geometry_msgs/Vector3 fix254
