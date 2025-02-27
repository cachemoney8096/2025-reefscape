package frc.robot.subsystems.elevator;

import frc.robot.Constants;

public class ElevatorCal {
  public static final double ELEVATOR_SCORE_P = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_SCORE_I = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_SCORE_D = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_SCORE_FF = Constants.PLACEHOLDER_DOUBLE;
    
  public static final double ELEVATOR_CLIMB_P = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_CLIMB_I = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_CLIMB_D = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_CLIMB_FF = Constants.PLACEHOLDER_DOUBLE;

  public static final double ELEVATOR_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS = 80.0,
      ELEVATOR_MOTOR_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 80.0;

  public static final double POSITION_INTAKE_INCHES = 21.75,
      POSITION_HOME_INCHES = 0.0,
      POSITION_SCORE_L4_INCHES = 32.49,
      POSITION_SCORE_L3_INCHES = 21.75,
      POSITION_SCORE_L2_INCHES = 10.0,
      POSITION_SCORE_L1_INCHES = 10.0,
      POSITION_ARM_CLEAR_OF_CLIMB_INCHES = 7.75,
      POSITION_SHALLOW_PREP_INCHES = Constants.PLACEHOLDER_DOUBLE,
      POSITION_SHALLOW_CLIMB_INCHES = Constants.PLACEHOLDER_DOUBLE;

  public static final double ELEVATOR_MARGIN_INCHES = 0.5,
      AT_CLEAR_POSITION_MARGIN = 0.5;

  public static final double TEST_ELEVATOR_MOVEMENT_VOLTAGE = 5.0;

  public static final double MAX_VELOCITY_IN_PER_SECOND_SCORE = 8.0,
      MAX_ACCELERATION_IN_PER_SECOND_SQUARED_SCORE = 10.0;

  public static final double MAX_VELOCITY_IN_PER_SECOND_CLIMB = 2.0,
      MAX_ACCELERATION_IN_PER_SECOND_SQUARED_CLIMB = 3.5;
}
