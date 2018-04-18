

What you have to know to allow HippoCapmus waypoint following in a desired depth.
Some of the parameters require the knowledge of the master's thesis "Position Estimation and Control of Autonomous Underwater Robots".


1. Run the following apps as

    path_contr WS
    ekf_position start
    trajectory_planner ellipse

2. You need the following parameters to influence the vehicle behaviour and change the sequence of waypoints

    path_contr parameters:

        Controller gains
            For the Pitch control input
                PC_DEPTH        --> set the depth setpoint
                PC_DEPTH_P
                PC_DEPTH_I
                PC_DEPTH_D
                PC_PITCH_DES_L  --> parameter L_Des see mester's thesis

             For the Roll control input
                PC_ROLL_GAIN        --> P Gain
                PC_ROLL_RATE_G      --> D Gain

             For the Yaw control input
                PC_YAW_GAIN        --> P Gain
                PC_YAW_RATE_G      --> D Gain

             For the Thrust control input
                PC_K_p              --> P Gain

        Other parameters

            PC_OG_THRUST_C  --> set a constant value for thrust if PC_PI_RO_ONLY == 1
            PC_OG_YAW_C     --> set a constant value for thrust if PC_PI_RO_ONLY == 1
            PC_OG_THRUST    --> scale thrust with the factor OG_THRUST
            PC_OG_YAW       --> scale yaw with the factor OG_YAW
            PC_OG_ROLL      --> scale ROLL with the factor OG_ROLL
            PC_OG_PITCH     --> scale PITCH with the factor OG_PITCH
            PC_PI_RO_ONLY   --> set to 0 to allow full control set to 1 to set a constant thurst and yaw
            PC_NO_BACK      --> set to 0 to allow backwards thrust set to 1 to allow only forward thrust
            PC_SCALE        --> first Scale tests to reduce saturation of motor signals for PC_SCALE = 0,1 or 2 (in master's thesis no such scaling so PC_SCALE is set to a value >2)




    ekf_position parameters:
        EKF_TRANS   --> set to 1 to transform vehicle's antennas position into body center coordinates


    trajectory_planner parameters:

    	TP_MAX_ERROR        --> set the error acceptance range as desicribed in the master's thesis
		TP_WP_SHAPE         --> set different waypoint sequences with TP_WP_SHAPE == 0...5