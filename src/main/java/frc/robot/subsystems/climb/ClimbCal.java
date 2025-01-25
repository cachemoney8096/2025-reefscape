package frc.robot.subsystems.climb;

import frc.robot.Constants;

public class ClimbCal {
    public static final double CLIMBING_P = Constants.PLACEHOLDER_DOUBLE,
        CLIMBING_I = Constants.PLACEHOLDER_DOUBLE,
        CLIMBING_D = Constants.PLACEHOLDER_DOUBLE,
        CLIMBING_FF = Constants.PLACEHOLDER_DOUBLE;

    public static final double POSITIONING_P = Constants.PLACEHOLDER_DOUBLE,
        POSITIONING_I = Constants.PLACEHOLDER_DOUBLE, 
        POSITIONING_D = Constants.PLACEHOLDER_DOUBLE,
        POSITIONING_FF = Constants.PLACEHOLDER_DOUBLE;

    public static final double CLIMB_CLIMBING_POSITION_DEGREES = Constants.PLACEHOLDER_DOUBLE,
        CLIMB_STOWED_POSITION_DEGREES = Constants.PLACEHOLDER_DOUBLE,
        CLIMB_CLEAR_OF_ARM_DEGREES = Constants.PLACEHOLDER_DOUBLE,
        CLIMB_CLIMBING_PREP_DEGREES = Constants.PLACEHOLDER_DOUBLE;

    public static final double CLIMB_MOTOR_MAX_VELOCITY_RPS = Constants.PLACEHOLDER_DOUBLE,
        CLIMB_MOTOR_MAX_ACCELERATION_RPS_SQUARED = Constants.PLACEHOLDER_DOUBLE;

    public static final double CLIMB_TALONS_SUPPLY_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE,
        CLIMB_TALONS_STATOR_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE;

    public static final double CLIMB_INTERFERENCE_THRESHOLD_MAX_DEGREES = Constants.PLACEHOLDER_DOUBLE,
        CLIMB_INTERFERENCE_THRESHOLD_MIN_DEGREES = Constants.PLACEHOLDER_DOUBLE;
    
    public static final double CLIMB_DESIRED_POSITION_ERROR_MARGIN_DEG = Constants.PLACEHOLDER_DOUBLE;

    public static final double CLIMB_DUTY_CYCLE_UPDATE_FREQ_HZ = Constants.PLACEHOLDER_DOUBLE;    

}
