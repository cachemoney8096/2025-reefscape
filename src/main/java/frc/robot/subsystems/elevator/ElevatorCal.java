package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class ElevatorCal {
    public static final double ELEVATOR_MOTOR_P = Constants.PLACEHOLDER_DOUBLE,
            ELEVATOR_MOTOR_I = Constants.PLACEHOLDER_DOUBLE,
            ELEVATOR_MOTOR_D = Constants.PLACEHOLDER_DOUBLE,
            ELEVATOR_MOTOR_FF = Constants.PLACEHOLDER_DOUBLE,
            ELEVATOR_CLIMB_P = Constants.PLACEHOLDER_DOUBLE,
            ELEVATOR_CLIMB_I = Constants.PLACEHOLDER_DOUBLE,
            ELEVATOR_CLIMB_D = Constants.PLACEHOLDER_DOUBLE,
            ELEVATOR_CLIMB_FF = Constants.PLACEHOLDER_DOUBLE;

    public static final double ELEVATOR_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE,
            ELEVATOR_MOTOR_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE;
    
    public static final double ELEVATOR_ROLLERS_INTAKING_SPEED_PERCENT = Constants.PLACEHOLDER_DOUBLE, 
            ELEVATOR_MOTOR_SCORING_SPEED_PERCENT = Constants.PLACEHOLDER_DOUBLE;
  
            public static final double POSITION_HOME_INCHES = Constants.PLACEHOLDER_DOUBLE,
            POSITION_SCORE_L4_INCHES = Constants.PLACEHOLDER_DOUBLE,
            POSITION_SCORE_L3_INCHES = Constants.PLACEHOLDER_DOUBLE,
            POSITION_SCORE_L2_INCHES = Constants.PLACEHOLDER_DOUBLE,
            POSITION_SCORE_L1_INCHES = Constants.PLACEHOLDER_DOUBLE,
            POSITION_PRE_SHALLOWCLIMB_INCHES = Constants.PLACEHOLDER_DOUBLE,
            POSITION_SCORE_SHALLOWCLIMB_INCHES = Constants.PLACEHOLDER_DOUBLE;

    public static final double MAX_VELOCITY_IN_PER_SECOND = Constants.PLACEHOLDER_DOUBLE,
            MAX_ACCELERATION_IN_PER_SECOND_SQUARED = Constants.PLACEHOLDER_DOUBLE;
    public static final double MAX_VELOCITY_IN_PER_SECOND_CLIMB = Constants.PLACEHOLDER_DOUBLE,
            MAX_ACCELERATION_IN_PER_SECOND_SQUARED_CLIMB = Constants.PLACEHOLDER_DOUBLE;

    public static final double ELEVATOR_KS = Constants.PLACEHOLDER_DOUBLE,
            ELEVATOR_KV = Constants.PLACEHOLDER_DOUBLE,
            ELEVATOR_KA = Constants.PLACEHOLDER_DOUBLE;
    
}
