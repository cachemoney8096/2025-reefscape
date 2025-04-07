package frc.robot.subsystems.elevator;

import frc.robot.Constants;

public class ElevatorCal {
  public static final double
      ELEVATOR_SCORE_P = 4.0, // 5.0 // TODO change these back after elevator is fixed
      ELEVATOR_SCORE_I = 0.0000, // 0.0005,
      ELEVATOR_SCORE_D = 0.0,
      ELEVATOR_SCORE_FF = 0.000; // 0.005
  public static final double ELEVATOR_CLIMB_P = 0.0001,
      ELEVATOR_CLIMB_I = 0.0,
      ELEVATOR_CLIMB_D = 0.0,
      ELEVATOR_CLIMB_FF = 0.001;
  public static final double ELEVATOR_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS = 80.0,
      ELEVATOR_MOTOR_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 80.0;
  public static final double POSITION_INTAKE_INCHES = 6.0, // 22.00,
      POSITION_HOME_INCHES = 6.0,
      POSITION_SCORE_L4_INCHES = 6.0,
      POSITION_SCORE_L3_INCHES = 18.0, // 21.75
      POSITION_SCORE_L2_INCHES = 6.0, // 10.0,
      POSITION_SCORE_L1_INCHES = 6.0,
      POSITION_ARM_CLEAR_OF_CLIMB_INCHES = 0.0,
      POSITION_ALGAE_INCHES = 10.0,
      POSITION_SHALLOW_PREP_INCHES = Constants.PLACEHOLDER_DOUBLE,
      POSITION_SHALLOW_CLIMB_INCHES = Constants.PLACEHOLDER_DOUBLE;
  public static final double ELEVATOR_MARGIN_INCHES = 0.5, AT_CLEAR_POSITION_MARGIN = 0.5;

  public static final double TEST_ELEVATOR_MOVEMENT_VOLTAGE = 5.0;

  public static final double MAX_VELOCITY_IN_PER_SECOND_SCORE = 8.0,
      MAX_ACCELERATION_IN_PER_SECOND_SQUARED_SCORE = 10.0;
  public static final double MAX_VELOCITY_IN_PER_SECOND_CLIMB = 2.0,
      MAX_ACCELERATION_IN_PER_SECOND_SQUARED_CLIMB = 3.5;
}
