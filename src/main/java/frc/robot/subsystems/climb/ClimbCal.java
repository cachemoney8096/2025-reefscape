package frc.robot.subsystems.climb;

import frc.robot.Constants;

public class ClimbCal {
  public static final double CLIMBING_P = 3.5,
      CLIMBING_I = 0.0,
      CLIMBING_D = 1.0,
      CLIMBING_FF = 0.001;

  public static final double POSITIONING_P = 3.5,
      POSITIONING_I = 0.0,
      POSITIONING_D = 1.0,
      POSITIONING_FF = 0.001;

  public static final double CLIMB_CLIMBING_POSITION_DEGREES = 77.8,
      CLIMB_STOWED_POSITION_DEGREES = 0.0,
      CLIMB_CLIMBING_PREP_DEGREES = 167.8;

  // TODO if climb doesnt work, this could be the cause
  public static final double CLIMB_MOTOR_MAX_VELOCITY_RPS = 0.5,
      CLIMB_MOTOR_MAX_ACCELERATION_RPS_SQUARED = 0.833;

  public static final double CLIMB_TALONS_SUPPLY_CURRENT_LIMIT_AMPS = 80.0,
      CLIMB_TALONS_STATOR_CURRENT_LIMIT_AMPS = 80.0;

  public static final double CLIMB_MARGIN_DEGREES = 0.5;
  public static final double TEST_CLIMB_MOVEMENT_VOLTAGE = 4.0;
}
