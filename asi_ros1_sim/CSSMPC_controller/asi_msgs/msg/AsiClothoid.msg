float64 start_x_m                         # start position in meters (x-coordinate)
float64 start_y_m		                      # start position in meters (y-coordinate)
float64 start_heading_rad                 # start heading in radians
float64 start_curvature_inv_m                   # curvature in inverse meters
float64 delta_curvature_per_length_inv_m2 # zero for arcs nonzero for clothoids
float64 length_m		                      # length in meters (can be negative for driving in reverse)
float64 speed_mps		                      # speed in meters per second (should only be positive values)