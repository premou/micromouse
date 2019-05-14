/*
 * controller_helper.c
 *
 *  Created on: 6 mai 2019
 *      Author: Patrick
 */

// reset all PID and setpoint
void reset_speed_control()
{
	// forward speed PID
	ctx.x_speed_target = 0;
	ctx.x_speed_setpoint = 0;
	ctx.x_speed_current = 0;
	ctx.x_speed_error = 0;
	ctx.x_speed_pwm = 0;
	pid_reset(&ctx.x_speed_pid);

	// rotation speed PID
	ctx.w_speed_target = 0;
	ctx.w_speed_setpoint = 0;
	ctx.w_speed_current = 0;
	ctx.w_speed_error = 0;
	ctx.w_speed_pwm = 0;
	pid_reset(&ctx.w_speed_pid);

	// wall following position PID
	ctx.wall_position_target = 0;
	ctx.wall_position_setpoint = 0;
	ctx.wall_position_current = 0;
	ctx.wall_position_error = 0;
	ctx.wall_position_pwm = 0;
	pid_reset(&ctx.wall_position_pid);

	// front wall distance position PID
	ctx.x_wall_front_target = 0;
	ctx.x_wall_front_setpoint = 0;
	ctx.x_wall_front_current = 0;
	ctx.x_wall_front_error = 0;
	ctx.x_wall_front_pwm = 0;
	pid_reset(&ctx.x_wall_front_pid);

	// front wall angle position PID
	ctx.w_wall_front_target = 0;
	ctx.w_wall_front_setpoint = 0;
	ctx.w_wall_front_current = 0;
	ctx.w_wall_front_error = 0;
	ctx.w_wall_front_pwm = 0;
	pid_reset(&ctx.w_wall_front_pid);

	motor_speed_left(0);
	motor_speed_right(0);
}

void speed_control(float x_speed_target, float w_speed_target)
{
	// forward speed PID
	ctx.x_speed_target = x_speed_target;
	ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.x_speed_setpoint);
	ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) * CONTROLLER_FREQ_F;
	ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
	ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

	// rotation speed PID
	ctx.w_speed_target = w_speed_target;
	ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.w_speed_setpoint);
	ctx.w_speed_current = gyro_get_dps();
	ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
	ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

	// wall following position PID
	ctx.wall_position_target = 0;
	ctx.wall_position_setpoint = 0;
	ctx.wall_position_current = 0;
	ctx.wall_position_error = 0;
	ctx.wall_position_pwm = 0;
	pid_reset(&ctx.wall_position_pid);

	// front wall distance position PID
	ctx.x_wall_front_target = 0;
	ctx.x_wall_front_setpoint = 0;
	ctx.x_wall_front_current = 0;
	ctx.x_wall_front_error = 0;
	ctx.x_wall_front_pwm = 0;
	pid_reset(&ctx.x_wall_front_pid);

	// front wall angle position PID
	ctx.w_wall_front_target = 0;
	ctx.w_wall_front_setpoint = 0;
	ctx.w_wall_front_current = 0;
	ctx.w_wall_front_error = 0;
	ctx.w_wall_front_pwm = 0;
	pid_reset(&ctx.w_wall_front_pid);

	motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm - W_SPEED_KF*w_speed_target);
	motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm + W_SPEED_KF*w_speed_target);
}

void speed_control_with_wall_following(float x_speed_target)
{
	// forward speed PID
	ctx.x_speed_target = x_speed_target;
	ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.x_speed_setpoint);
	ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) * CONTROLLER_FREQ_F;
	ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
	ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

	// rotation speed PID
	ctx.w_speed_target = 0;
	ctx.w_speed_setpoint = 0;
	ctx.w_speed_current = 0;
	ctx.w_speed_error = 0;
	ctx.w_speed_pwm = 0;
	pid_reset(&ctx.w_speed_pid);

	// wall following position PID
	ctx.wall_position_target = 0.0F;
	ctx.wall_position_setpoint = 0.0F;
	ctx.wall_position_current = wall_sensor_get_side_error();
	ctx.wall_position_error = ctx.wall_position_setpoint - ctx.wall_position_current;
	ctx.wall_position_pwm = pid_output(&ctx.wall_position_pid, ctx.wall_position_error);

	// front wall distance position PID
	ctx.x_wall_front_target = 0;
	ctx.x_wall_front_setpoint = 0;
	ctx.x_wall_front_current = 0;
	ctx.x_wall_front_error = 0;
	ctx.x_wall_front_pwm = 0;
	pid_reset(&ctx.x_wall_front_pid);

	// front wall angle position PID
	ctx.w_wall_front_target = 0;
	ctx.w_wall_front_setpoint = 0;
	ctx.w_wall_front_current = 0;
	ctx.w_wall_front_error = 0;
	ctx.w_wall_front_pwm = 0;
	pid_reset(&ctx.w_wall_front_pid);

	motor_speed_left(ctx.x_speed_pwm - ctx.wall_position_pwm);
	motor_speed_right(ctx.x_speed_pwm + ctx.wall_position_pwm);
}

void speed_control_with_front_wall_calibration()
{
	// forward speed PID
	ctx.x_speed_target = 0;
	ctx.x_speed_setpoint = 0;
	ctx.x_speed_current = 0;
	ctx.x_speed_error = 0;
	ctx.x_speed_pwm = 0;
	pid_reset(&ctx.x_speed_pid);

	// rotation speed PID
	ctx.w_speed_target = 0;
	ctx.w_speed_setpoint = 0;
	ctx.w_speed_current = 0;
	ctx.w_speed_error = 0;
	ctx.w_speed_pwm = 0;
	pid_reset(&ctx.w_speed_pid);

	// wall following position PID
	ctx.wall_position_target = 0;
	ctx.wall_position_setpoint = 0;
	ctx.wall_position_current = 0;
	ctx.wall_position_error = 0;
	ctx.wall_position_pwm = 0;
	pid_reset(&ctx.wall_position_pid);

	// front wall distance position PID
	ctx.x_wall_front_target = WALL_FRONT_DISTANCE_mm; // mm
	ctx.x_wall_front_setpoint = WALL_FRONT_DISTANCE_mm; // mm
	ctx.x_wall_front_current = ( wall_sensor_get_dist(WALL_SENSOR_LEFT_STRAIGHT) + wall_sensor_get_dist(WALL_SENSOR_RIGHT_STRAIGHT) ) / 2.0;
	ctx.x_wall_front_error = ctx.x_wall_front_setpoint - ctx.x_wall_front_current;
	ctx.x_wall_front_pwm = pid_output(&ctx.x_wall_front_pid, ctx.x_wall_front_error);

	// front wall angle position PID
	ctx.w_wall_front_target = WALL_FRONT_ANGLE_mm; // mm
	ctx.w_wall_front_setpoint = WALL_FRONT_ANGLE_mm; // mm
	ctx.w_wall_front_current = wall_sensor_get_dist(WALL_SENSOR_LEFT_STRAIGHT) - wall_sensor_get_dist(WALL_SENSOR_RIGHT_STRAIGHT);
	ctx.w_wall_front_error = ctx.w_wall_front_setpoint - ctx.w_wall_front_current;
	ctx.w_wall_front_pwm = pid_output(&ctx.w_wall_front_pid, ctx.w_wall_front_error);


	motor_speed_left(constraint(-ctx.x_wall_front_pwm - ctx.w_wall_front_pwm,-30,30));
	motor_speed_right(constraint(-ctx.x_wall_front_pwm + ctx.w_wall_front_pwm,-30,30));
}


// reset longitudinal calibration (wall-to-no-wall, post-to-no-post)
void calibration_reset()
{
	ctx.calibration_state = CALIBRATION_IDLE;
	filter_reset(&ctx.filter_calibration_wall_distance_left);
	filter_reset(&ctx.filter_calibration_wall_distance_right);
}


bool calibration_update()
{
	float calibration_wall_distance_left = filter_output(&ctx.filter_calibration_wall_distance_left,wall_sensor_get_dist(WALL_SENSOR_LEFT_DIAG));
	float calibration_wall_distance_right = filter_output(&ctx.filter_calibration_wall_distance_right,wall_sensor_get_dist(WALL_SENSOR_RIGHT_DIAG));
	switch(ctx.calibration_state)
	{
	case CALIBRATION_IDLE:
		{
			if(encoder_get_absolute()>0.03f) // skip first 3 centimeter of current cell (position error)
			{
				if(calibration_wall_distance_left<120 && calibration_wall_distance_right<120)
				{
					ctx.calibration_state = CALIBRATION_WALL_BOTH;
				}
				else if(calibration_wall_distance_left<120)
				{
					ctx.calibration_state = CALIBRATION_WALL_LEFT;
				}
				else if(calibration_wall_distance_right<120)
				{
					ctx.calibration_state = CALIBRATION_WALL_RIGHT;
				}
				else
				{
					ctx.calibration_state = CALIBRATION_NO_WALL;
				}
			}
		}
		break;
	case CALIBRATION_NO_WALL:
		{
			if(calibration_wall_distance_left<140 && calibration_wall_distance_right<140)
			{
				ctx.calibration_state = CALIBRATION_POST_BOTH;
			}
			else if(calibration_wall_distance_left<140)
			{
				ctx.calibration_state = CALIBRATION_POST_LEFT;
			}
			else if(calibration_wall_distance_right<140)
			{
				ctx.calibration_state = CALIBRATION_POST_RIGHT;
			}
		}
		break;
	case CALIBRATION_POST_LEFT:
		{
			if(calibration_wall_distance_left>140)
			{
				if(abs(encoder_get_absolute()-REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST)<=30) // FIX : blend behavior
					encoder_set_absolute(REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_right<140)
			{
				ctx.calibration_state = CALIBRATION_POST_BOTH;
			}
		}
		break;
	case CALIBRATION_POST_RIGHT:
		{
			if(calibration_wall_distance_right>140)
			{
				if(abs(encoder_get_absolute()-REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST)<=30) // FIX : blend behavior
					encoder_set_absolute(REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_left<140)
			{
				ctx.calibration_state = CALIBRATION_POST_BOTH;
			}
		}
		break;
	case CALIBRATION_POST_BOTH:
		{
			if(calibration_wall_distance_right>140 || calibration_wall_distance_left>140)
			{
				if(abs(encoder_get_absolute()-REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST)<=30) // FIX : blend behavior
					encoder_set_absolute(REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
		}
		break;
	case CALIBRATION_WALL_LEFT:
		{
			if(calibration_wall_distance_left>140)
			{
				if(abs(encoder_get_absolute()-REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL)<=30) // FIX : blend behavior
					encoder_set_absolute(REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_right<140)
			{
				ctx.calibration_state = CALIBRATION_WALL_LEFT_with_RIGHT_POST;
			}
		}
		break;
	case CALIBRATION_WALL_RIGHT:
		{
			if(calibration_wall_distance_right>140)
			{
				if(abs(encoder_get_absolute()-REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL)<=30) // FIX : blend behavior
					encoder_set_absolute(REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_left<140)
			{
				ctx.calibration_state = CALIBRATION_WALL_RIGHT_with_LEFT_POST;
			}
		}
		break;
	case CALIBRATION_WALL_BOTH:
		{
			if(calibration_wall_distance_right>140 || calibration_wall_distance_left>140)
			{
				if(abs(encoder_get_absolute()-REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL)<=30) // FIX : blend behavior
					encoder_set_absolute(REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
		}
		break;
	case CALIBRATION_WALL_LEFT_with_RIGHT_POST:
		{
			if(calibration_wall_distance_left>140)
			{
				if(abs(encoder_get_absolute()-REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL)<=30) // FIX : blend behavior
					encoder_set_absolute(REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_right>140)
			{
				if(abs(encoder_get_absolute()-REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST)<=30) // FIX : blend behavior
					encoder_set_absolute(REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}

		}
		break;
	case CALIBRATION_WALL_RIGHT_with_LEFT_POST:
		{
			if(calibration_wall_distance_right>140)
			{
				if(abs(encoder_get_absolute()-REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL)<=30) // FIX : blend behavior
					encoder_set_absolute(REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_left>140)
			{
				if(abs(encoder_get_absolute()-REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST)<=30) // FIX : blend behavior
					encoder_set_absolute(REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}

		}
		break;

	case CALIBRATION_END:
		{

		}
		break;
	}
	return false;
}

// Recorded moves
action_t actions_scenario[] =
{
#ifdef SC_START_STOP
		ACTION_STOP,
#endif
#ifdef SC_START_RUN_STOP
		ACTION_RUN_1,
		ACTION_STOP,
#endif
#ifdef SC_START_RUN2_STOP
		ACTION_RUN_1,
		ACTION_RUN_1,
		ACTION_STOP,
#endif
#ifdef SC_START_RUN3_STOP
		ACTION_RUN_1,
		ACTION_RUN_1,
		ACTION_RUN_1,
		ACTION_STOP,
#endif
#ifdef SC_TURN_RIGHT_TEST
		ACTION_TURN_RIGHT,
		ACTION_STOP,
#endif
#ifdef SC_SQUARE_TEST_1_TURN
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_STOP,
#endif
#ifdef SC_SQUARE_TEST_2_TURN
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,

		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_STOP,
#endif
#ifdef SC_U_TURN
		ACTION_U_TURN_RIGHT,
		ACTION_STOP,
#endif
#ifdef SC_RUN1_UTURN_RUN1
		ACTION_RUN_1,
		ACTION_U_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_STOP,
#endif
#ifdef SC_F_L90_F
		ACTION_RUN_1,
		ACTION_TURN_LEFT,
		ACTION_RUN_1,
		ACTION_STOP,
#endif
#ifdef SC_F_R90_F
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_STOP,
#endif
#ifdef SC_F_R90_R90_F
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_STOP,
#endif
#ifdef SC_F_R90_F_R90_F
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_TURN_RIGHT,
		ACTION_RUN_1,
		ACTION_STOP,
#endif
#ifdef SC_ZIGZAG
		ACTION_TURN_RIGHT,
		ACTION_TURN_LEFT,
		ACTION_TURN_RIGHT,
		ACTION_TURN_LEFT,
		ACTION_TURN_RIGHT,
		ACTION_TURN_LEFT,
		ACTION_STOP,
#endif
#ifdef RAND_MOVES_3
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_STOP,
#endif
#ifdef RAND_MOVES_6
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_STOP,
#endif
#ifdef RAND_MOVES_12
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_STOP,
#endif
#ifdef RAND_MOVES_16
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_RAND,
		ACTION_STOP,
#endif
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,
//		ACTION_RAND,

// ALWAY HERE at the end
		ACTION_IDLE
};

// play recorded moves
action_t get_next_move()
{
	action_t next = actions_scenario[ctx.actions_index++];

	//HAL_Serial_Print(&com,"\nget_next_move() returns %d\n", actions_scenario[ctx.actions_index]);
	if(next==ACTION_RAND)
	{
		float r = (float)(rand())/(float)RAND_MAX;

		if(!wall_sensor_is_front_wall_detected() &&
			!wall_sensor_is_left_wall_detected() &&
			 !wall_sensor_is_right_wall_detected() )
		{
			if(r<0.25)
			{
				next = ACTION_TURN_LEFT;
			}
			else if(r<0.75)
			{
				next = ACTION_RUN_1;
			}
			else
			{
				next = ACTION_TURN_RIGHT;
			}
		}
		else if(wall_sensor_is_front_wall_detected() &&
				!wall_sensor_is_left_wall_detected() &&
				!wall_sensor_is_right_wall_detected() )
		{
			if(r<0.5)
			{
				next = ACTION_TURN_LEFT;
			}
			else
			{
				next = ACTION_TURN_RIGHT;
			}
		}
		else if(!wall_sensor_is_front_wall_detected() &&
				wall_sensor_is_left_wall_detected() &&
				!wall_sensor_is_right_wall_detected() )
		{
			if(r<0.6)
			{
				next = ACTION_RUN_1;
			}
			else
			{
				next = ACTION_TURN_RIGHT;
			}
		}
		else if(!wall_sensor_is_front_wall_detected() &&
				!wall_sensor_is_left_wall_detected() &&
				wall_sensor_is_right_wall_detected() )
		{
			if(r<0.6)
			{
				next = ACTION_RUN_1;
			}
			else
			{
				next = ACTION_TURN_LEFT;
			}
		}
		else if(!wall_sensor_is_front_wall_detected() &&
				wall_sensor_is_left_wall_detected() &&
				wall_sensor_is_right_wall_detected() )
		{
			next = ACTION_RUN_1;
		}
		else if(wall_sensor_is_front_wall_detected() &&
				!wall_sensor_is_left_wall_detected() &&
				wall_sensor_is_right_wall_detected() )
		{
			next = ACTION_TURN_LEFT;
		}
		else if(wall_sensor_is_front_wall_detected() &&
				wall_sensor_is_left_wall_detected() &&
				!wall_sensor_is_right_wall_detected() )
		{
			next = ACTION_TURN_RIGHT;
		}
		else
		{
			next = ACTION_U_TURN_RIGHT;
		}
	}
	return next;
}
