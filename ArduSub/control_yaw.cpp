#include "Sub.h"

bool Sub::yaw_init()
{
	pos_control.set_alt_target(0);
    if (prev_control_mode == ALT_HOLD) {
        last_roll = ahrs.roll_sensor;
        last_pitch = ahrs.pitch_sensor;
    } else {
        last_roll = 0;
        last_pitch = 0;
    }
    last_yaw = ahrs.yaw_sensor;
	return true;
}

void Sub::yaw_run()
{
	if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        last_roll = 0;
        last_pitch = 0;
        last_yaw = ahrs.yaw_sensor;
        return;
    }
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

//    handle_attitude();
/*
	Vector3f asv_destination_neu,asv_curr_pos_nue;
	float target_yaw;

	
	Location asv_curr_pos;
	asv_curr_pos.lat = current_loc.lat;
	asv_curr_pos.lng = current_loc.lng;
	asv_curr_pos.alt = 0;
//	36.162411,120.496158

	Location asv_destination;
	asv_destination.lat = 36.162411;
	asv_destination.lng = 120.496158;
	asv_destination.alt = 0;

	asv_destination_neu = wp_nav.asv_yaw_destination_trans(asv_destination);
	asv_curr_pos_nue = wp_nav.asv_yaw_destination_trans(asv_curr_pos);
	target_yaw = get_bearing_cd(asv_curr_pos_nue, asv_destination_neu);
*/

	
	attitude_control.asv_input_euler_angle_yaw(g.heading_control * 100);

	motors.set_forward(channel_forward->norm_input());
    	motors.set_lateral(0);




}
