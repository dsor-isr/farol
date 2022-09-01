Header  header                #Timestamp is the "measurement time" from the manual.
uint8   id                    #Acoustic ID of the referenced node.
string  role                  #MORPH role of the referenced node.
float64 bearing_rel           #[radians] Bearing of target relative to modem frame.
float64 elevation_rel         #[radians] Elevation of target relative to modem frame.
float64 bearing_comp          #[radians] Orientation compensated bearing to target.
float64 elevation_comp        #[radians] Orientation compensated elevation to target.
float64 roll                  #[radians] Roll from MTi-G
float64 pitch                 #[radians] Pitch from MTi-G
float64 yaw                   #[radians] Yaw from MTi-G
float64 rssi                  #[dB] Received Signal Strength Indicator
uint8   integrity             # Signal integrity
float64 accuracy              #[radians] Estimation accuracy.
