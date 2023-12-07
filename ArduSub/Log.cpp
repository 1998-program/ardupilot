#include "Sub.h"

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from AP_Logger log memory
// Code to interact with the user to dump or erase logs

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float  throttle_in;
    float  angle_boost;
    float    throttle_out;
    float    throttle_hover;
    float    desired_alt;
    float    inav_alt;
    float    baro_alt;
    int16_t  desired_rangefinder_alt;
    int16_t  rangefinder_alt;
    float    terr_alt;
    int16_t  target_climb_rate;
    int16_t  climb_rate;
};

// Write a control tuning packet
void Sub::Log_Write_Control_Tuning()
{
    // get terrain altitude
    float terr_alt = 0.0f;
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    terrain.height_above_terrain(terr_alt, true);
#endif

    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control.get_throttle_in(),
        angle_boost         : attitude_control.angle_boost(),
        throttle_out        : motors.get_throttle(),
        throttle_hover      : motors.get_throttle_hover(),
        desired_alt         : pos_control.get_alt_target() / 100.0f,
        inav_alt            : inertial_nav.get_altitude() / 100.0f,
        baro_alt            : barometer.get_altitude(),
        desired_rangefinder_alt   : (int16_t)target_rangefinder_alt,
        rangefinder_alt           : rangefinder_state.alt_cm,
        terr_alt            : terr_alt,
        target_climb_rate   : (int16_t)pos_control.get_vel_target_z(),
        climb_rate          : climb_rate
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void Sub::Log_Write_Attitude()
{
    Vector3f targets = attitude_control.get_att_target_euler_cd();
    targets.z = wrap_360_cd(targets.z);
    logger.Write_Attitude(ahrs, targets);

    AP::ahrs_navekf().Log_Write();
    logger.Write_AHRS2(ahrs);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE();
#endif
    logger.Write_POS(ahrs);
}

struct PACKED log_MotBatt {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   lift_max;
    float   bat_volt;
    float   bat_res;
    float   th_limit;
};

// Write an rate packet
void Sub::Log_Write_MotBatt()
{
    struct log_MotBatt pkt_mot = {
        LOG_PACKET_HEADER_INIT(LOG_MOTBATT_MSG),
        time_us         : AP_HAL::micros64(),
        lift_max        : (float)(motors.get_lift_max()),
        bat_volt        : (float)(motors.get_batt_voltage_filt()),
        bat_res         : (float)(battery.get_resistance()),
        th_limit        : (float)(motors.get_throttle_limit())
    };
    logger.WriteBlock(&pkt_mot, sizeof(pkt_mot));
}

// Wrote an event packet
void Sub::Log_Write_Event(Log_Event id)
{
    logger.Write_Event(id);
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
UNUSED_FUNCTION
void Sub::Log_Write_Data(uint8_t id, int16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}


// struct PACKED log_ASV_ROBUST{
//     LOG_PACKET_HEADER;
//     uint64_t time_us;
//     float ASV_robust_yaw_force;
//     float ASV_robust_x_force;
//     float ASV_robust_y_force;
//     float ASV_robust_yaw_error;
//     float ASV_robust_x_error;
//     float ASV_robust_y_error;
// };

// void Sub::Log_write_ASV(){
//     struct log_ASV_ROBUST pkt_asv = {
//         LOG_PACKET_HEADER_INIT(LOG_ASV_MSG),
//         time_us : AP_HAL::micros64(),
//         ASV_robust_yaw_force : asv_yaw_robust_force,
//         ASV_robust_x_force : asv_x_robust_force,
//         ASV_robust_y_force : asv_y_robust_force,
//         ASV_robust_yaw_error : asv_yaw_robust_error_rad,
//         ASV_robust_x_error : asv_x_robust_error,
//         ASV_robust_y_error : asv_y_robust_error
//     };
//     logger.WirteBlock(&pkt_asv,sizeof(pkt_asv));
// }


struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
UNUSED_FUNCTION
void Sub::Log_Write_Data(uint8_t id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
void Sub::Log_Write_Data(uint8_t id, int32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            time_us  : AP_HAL::micros64(),
            id          : id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

// struct PACKED log_PID
// {
//     LOG_PACK
// }

struct PACKED log_ASV_Robust
{
    /* data */
    LOG_PACKET_HEADER; //日志包头
    uint64_t time_us;  //数据项
    float ASV_X_force_ROBUST;
    float ASV_Y_force_ROBUST;
    float ASV_YAW_force_ROBUST;
    float ASV_X_error_ROBUST;
    float ASV_Y_error_ROBUST;
    float ASV_YAW_error_ROBUST;
    float ASV_X_current_ROBUST;
    float ASV_Y_current_ROBUST;
    float ASV_YAW_current_ROBUST;
    float ASV_X_target_ROBUST;
    float ASV_Y_target_ROBUST;
    float ASV_YAW_target_ROBUST;
};

void Sub::Log_write_ASV_Robust()
{
    struct log_ASV_Robust pkt_asv_robust = {
        LOG_PACKET_HEADER_INIT(LOG_ASV_ROBUST_MSG),
        time_us : AP_HAL::micros64(),
        ASV_X_force_ROBUST : asv_x_robust_force,
        ASV_Y_force_ROBUST : asv_y_robust_force,
        ASV_YAW_force_ROBUST : asv_yaw_robust_force,
        ASV_X_error_ROBUST : asv_x_robust_error_b,
        ASV_Y_error_ROBUST : asv_y_robust_error_b,
        ASV_YAW_error_ROBUST : asv_yaw_robust_error_rad,
        ASV_X_current_ROBUST : inertial_nav.get_position().x/100,
        ASV_Y_current_ROBUST : inertial_nav.get_position().y/100,
        ASV_YAW_current_ROBUST : radians(AP::gps().yaw()),
        ASV_X_target_ROBUST : pos_control.get_pos_target_x()/100,
        ASV_Y_target_ROBUST : pos_control.get_pos_target_y()/100,
        ASV_YAW_target_ROBUST : asv_get_yaw()/100,
    };
    logger.WriteBlock(&pkt_asv_robust, sizeof(pkt_asv_robust));
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
void Sub::Log_Write_Data(uint8_t id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            time_us     : AP_HAL::micros64(),
            id          : id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    float data_value;
};

// Write a float data packet
UNUSED_FUNCTION
void Sub::Log_Write_Data(uint8_t id, float value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            time_us     : AP_HAL::micros64(),
            id          : id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

// logs when baro or compass becomes unhealthy
void Sub::Log_Sensor_Health()
{
    // check baro
    if (sensor_health.baro != barometer.healthy()) {
        sensor_health.baro = barometer.healthy();
        AP::logger().Write_Error(LogErrorSubsystem::BARO, (sensor_health.baro ? LogErrorCode::ERROR_RESOLVED : LogErrorCode::UNHEALTHY));
    }

    // check compass
    if (sensor_health.compass != compass.healthy()) {
        sensor_health.compass = compass.healthy();
        AP::logger().Write_Error(LogErrorSubsystem::COMPASS, (sensor_health.compass ? LogErrorCode::ERROR_RESOLVED : LogErrorCode::UNHEALTHY));
    }
}

struct PACKED log_GuidedTarget {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float pos_target_x;
    float pos_target_y;
    float pos_target_z;
    float vel_target_x;
    float vel_target_y;
    float vel_target_z;
};

// Write a Guided mode target
void Sub::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target)
{
    struct log_GuidedTarget pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GUIDEDTARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : target_type,
        pos_target_x    : pos_target.x,
        pos_target_y    : pos_target.y,
        pos_target_z    : pos_target.z,
        vel_target_x    : vel_target.x,
        vel_target_y    : vel_target.y,
        vel_target_z    : vel_target.z
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Sub::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qfffffffccfhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00BBBBBB" },
    { LOG_MOTBATT_MSG, sizeof(log_MotBatt),
      "MOTB", "Qffff",  "TimeUS,LiftMax,BatVolt,BatRes,ThLimit", "s-vw-", "F-00-" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "QBh",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "QBH",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "QBi",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "QBI",         "TimeUS,Id,Value", "s--", "F--" },
    // { LOG_ASV_MSG, sizeof(log_ASV_ROBUST),
    //  "HC_ANGLE", "Qff", "TimeUS,Value,Value", "s--", "F--"},
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "QBf",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_ASV_ROBUST_MSG, sizeof(log_ASV_Robust),
      "ASV_ROBSUST", "Qffffffffffff", "TimeUS, xF, yF, yawF, xErr, yErr, yawErr, xCur, yCur, yawCur, xTar, yTar, yawTar", "s------------", "F000000000000"},
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUID",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ", "s-mmmnnn", "F-000000" },
};

void Sub::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    logger.Write_Mode(control_mode, control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}


void Sub::log_init()
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else // LOGGING_ENABLED

void Sub::Log_Write_Control_Tuning() {}
void Sub::Log_Write_Performance() {}
void Sub::Log_Write_Attitude(void) {}
void Sub::Log_Write_MotBatt() {}
void Sub::Log_Write_Event(Log_Event id) {}
void Sub::Log_Write_Data(uint8_t id, int32_t value) {}
void Sub::Log_Write_Data(uint8_t id, uint32_t value) {}
void Sub::Log_Write_Data(uint8_t id, int16_t value) {}
void Sub::Log_Write_Data(uint8_t id, uint16_t value) {}
void Sub::Log_Write_Data(uint8_t id, float value) {}
void Sub::Log_Sensor_Health() {}
void Sub::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target) {}
void Sub::Log_Write_Vehicle_Startup_Messages() {}

void Sub::log_init(void) {}

#endif // LOGGING_ENABLED
