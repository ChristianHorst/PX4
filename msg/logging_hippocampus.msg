# Logging data for HippoCampus

float32 x		# x-position in meters NED
float32 y		# y-position in meters NED
float32 z		# z-position in meters NED

float32 xd		# desired x-position in meters NED
float32 yd		# desired y-position in meters NED
float32 zd		# desired z-position in meters NED

float32[3] xa   # orientation x-axis in NED
float32[3] ya   # orientation y-axis in NED
float32[3] za   # orientation z-axis in NED

float32[3] xad  # desired orientation x-axis in NED
float32[3] yad  # desired orientation y-axis in NED
float32[3] zad  # desired orientation z-axis in NED

float32 xvc		# flow velocity x-direction in meters/s NED
float32 yvc		# flow velocity y-direction in meters/s NED
float32 zvc		# flow velocity z-direction in meters/s NED

float32[3] e_r	# rotation error roll

float32 yaw		# yaw in rad NED
float32[3] e_p	# position error
float32[3] e_v	# velocity error
float32[3] e_w	# angular velocity error
float32[4] u_in	# actuator input
float32[4] u_out	# actuator input

float32 t       # time
