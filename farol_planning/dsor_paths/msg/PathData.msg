Header header          # Headers for the message and sequence ID with consecutively increasing ID
float64 gamma          # The gamma parameter used for the computations
float64[3] pd          # The path position, derivative and second derivative
float64[3] d_pd
float64[3] dd_pd
float64 curvature      # The curvature, tangent norm and derivative norm
float64 tangent
float64 derivative_norm
float64 vd             # The speeds desired for the path parameter
float64 d_vd
float64 vehicle_speed
float64 gamma_min      # The minimum and maximum value that gamma can achieve for the current path
float64 gamma_max
