Header      header                # Timestamp is the "measurement time" from the manual.
comms_msgs/Node source                # The transmitting node.
# From manual ( dated June 2014, 1.7.B ):
# USBLPHYD,<current time>,<measurement time>,<remote address>,<fix type>
# <delay 1-5>,<delay 2-5>,<delay 3-5>,<delay 4-5>,
# <delay 1-2>,<delay 4-1>,<delay 3-2>,<delay 3-4>
#
time[8]    delays                # [ns] Time-difference of arrivals between each hydrophone.

