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
  public static final double CLIMB_CLIMBING_POSITION_DEGREES = 77.8,
      CLIMB_STOWED_POSITION_DEGREES = 0,
      CLIMB_CLIMBING_PREP_DEGREES = 167.8;
  public static final double CLIMB_MOTOR_MAX_VELOCITY_RPS = 0.5,
      /** preliminary value: took 1/3 of arm acc. */
      CLIMB_MOTOR_MAX_ACCELERATION_RPS_SQUARED = 0.833;
  public static final double CLIMB_TALONS_SUPPLY_CURRENT_LIMIT_AMPS = 80.0,
      CLIMB_TALONS_STATOR_CURRENT_LIMIT_AMPS = 80.0;

  public static final double CLIMB_MARGIN_DEGREES = Constants.PLACEHOLDER_DOUBLE;
  public static final double TEST_CLIMB_MOVEMENT_VOLTAGE = Constants.PLACEHOLDER_DOUBLE;

  public static final double CLIMB_DUTY_CYCLE_UPDATE_FREQ_HZ = 50.0;
}
