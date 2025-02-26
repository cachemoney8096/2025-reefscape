package frc.robot.subsystems.elevator;

import frc.robot.Constants;

public class ElevatorCal {
  public static final int ELEVATOR_LIMIT_SWITCH_DIO_HOME = Constants.PLACEHOLDER_INT,
      ELEVATOR_LIMIT_SWITCH_DIO_BELOWHOME = Constants.PLACEHOLDER_INT,
      ELEVATOR_LIMIT_SWITCH_DIO_TOP = Constants.PLACEHOLDER_INT;
  public static final double ELEVATOR_SCORE_P = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_SCORE_I = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_SCORE_D = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_SCORE_FF = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_CLIMB_P = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_CLIMB_I = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_CLIMB_D = Constants.PLACEHOLDER_DOUBLE,
      ELEVATOR_CLIMB_FF = Constants.PLACEHOLDER_DOUBLE;
  public static final double ELEVATOR_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS = 80.0,
      ELEVATOR_MOTOR_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 80.0;
  public static final double POSITION_INTAKE_INCHES = 21.75,
      POSITION_HOME_INCHES = 0.0,
      /**
       * L4 is Absolute Max. We need to test this thoroughly to ensure motors don't stall when
       * approaching max.
       */
      POSITION_SCORE_L4_INCHES = 32.49,
      POSITION_SCORE_L3_INCHES = 21.75,
      POSITION_SCORE_L2_INCHES = 10.0,
      POSITION_SCORE_L1_INCHES = 10.0,
      POSITION_ARM_CLEAR_OF_CLIMB_INCHES = 7.75,
      POSITION_ALGAE_INCHES = Constants.PLACEHOLDER_DOUBLE,

      // TODO: determine if shallow climb is truly being implemented or not (most likely not)
      POSITION_SHALLOW_PREP_INCHES = Constants.PLACEHOLDER_DOUBLE,
      POSITION_SHALLOW_CLIMB_INCHES = Constants.PLACEHOLDER_DOUBLE;

  public static final double ELEVATOR_MARGIN_DEGREES = Constants.PLACEHOLDER_DOUBLE;

  // Trapezoid Motion Profiling
  public static final double MAX_VELOCITY_IN_PER_SECOND_SCORE = 40.0,
      MAX_ACCELERATION_IN_PER_SECOND_SQUARED_SCORE = 640.0;
  public static final double MAX_VELOCITY_IN_PER_SECOND_CLIMB = Constants.PLACEHOLDER_DOUBLE,
      MAX_ACCELERATION_IN_PER_SECOND_SQUARED_CLIMB = Constants.PLACEHOLDER_DOUBLE;

  public static final double AT_CLEAR_POSITION_MARGIN = 0.5;
}
