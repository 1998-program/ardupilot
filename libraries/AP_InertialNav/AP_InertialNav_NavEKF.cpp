#include <AP_HAL/AP_HAL.h>
#include "AP_InertialNav.h"

#if AP_AHRS_NAVEKF_AVAILABLE
extern const AP_HAL::HAL &hal;
/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */

/**
   update internal state
*/
void AP_InertialNav_NavEKF::update()
{
    // get the NE position relative to the local earth frame origin
    Vector2f posNE;
    if (_ahrs_ekf.get_relative_position_NE_origin(posNE)) {
        _relpos_cm.x = posNE.x * 100 ; // convert from m to cm
        _relpos_cm.y = posNE.y * 100 ; // convert from m to cm
        //hal.uartC->printf("%f\n",_relpos_cm.x);
    }

    float gps_yaw;
    if (_ahrs_ekf.get_gps_yaw(gps_yaw)) {
        // if(gps_yaw <= 340 && gps_yaw >= 0){
        //     _gps_yaw = gps_yaw + 20;
        // }
        // if(gps_yaw <= 360 && gps_yaw > 340){
        //     _gps_yaw = gps_yaw - 340;
        // }
        
        _gps_yaw = gps_yaw;
       //hal.uartC->printf("yaw:%f\n",_gps_yaw);
    }

    // get the D position relative to the local earth frame origin
    float posD;
    if (_ahrs_ekf.get_relative_position_D_origin(posD)) {
        _relpos_cm.z = - posD * 100; // convert from m in NED to cm in NEU
    }

    // get the velocity relative to the local earth frame
    Vector3f velNED;
    if (_ahrs_ekf.get_velocity_NED(velNED)) {
        _velocity_cm = velNED * 100; // convert to cm/s
        _velocity_cm.z = -_velocity_cm.z; // convert from NED to NEU
    }
}

/**
 * get_filter_status : returns filter status as a series of flags
 */
nav_filter_status AP_InertialNav_NavEKF::get_filter_status() const
{
    nav_filter_status status;
    _ahrs_ekf.get_filter_status(status);
    return status;
}

/**
 * get_position - returns the current position relative to the home location in cm.
 *
 * @return
 */
const Vector3f &AP_InertialNav_NavEKF::get_position(void) const 
{
    return _relpos_cm;
}

const float &AP_InertialNav_NavEKF::get_gpsyaw(void) const 
{
    return _gps_yaw;
}
/**
 * get_velocity - returns the current velocity in cm/s
 *
 * @return velocity vector:
 *      		.x : latitude  velocity in cm/s
 * 				.y : longitude velocity in cm/s
 * 				.z : vertical  velocity in cm/s
 */
const Vector3f &AP_InertialNav_NavEKF::get_velocity() const
{
    return _velocity_cm;
}

/**
 * get_speed_xy - returns the current horizontal speed in cm/s
 *
 * @returns the current horizontal speed in cm/s
 */
float AP_InertialNav_NavEKF::get_speed_xy() const
{
    return norm(_velocity_cm.x, _velocity_cm.y);
}

/**
 * get_altitude - get latest altitude estimate in cm
 * @return
 */
float AP_InertialNav_NavEKF::get_altitude() const
{
    return _relpos_cm.z;
}

/**
 * get_velocity_z - returns the current climbrate.
 *
 * @see get_velocity().z
 *
 * @return climbrate in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_z() const
{
    return _velocity_cm.z;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
