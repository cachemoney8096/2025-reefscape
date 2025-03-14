package frc.robot.subsystems.climb;

public class ClimbCal {
  public static final double CLIMBING_P = 0.01,
      CLIMBING_I = 0.0,
      CLIMBING_D = 0.0,
      CLIMBING_FF = 0.0;
  public static final double POSITIONING_P = 0.05,
      POSITIONING_I = 0.0,
      POSITIONING_D = 0.0,
      POSITIONING_FF = 0.0;
//   public static final double CLIMB_CLIMBING_POSITION_DEGREES = 77.8 + 129.5,
//       CLIMB_STOWED_POSITION_DEGREES = 0.0 + 129.5,
//       CLIMB_CLIMBING_PREP_DEGREES = 167.8 + 129.5;
public static final double CLIMB_CLIMBING_POSITION_DEGREES = 85,
      CLIMB_STOWED_POSITION_DEGREES = 308,
      CLIMB_CLIMBING_PREP_DEGREES = 76;

  // TODO if climb doesnt work, this could be the cause
  public static final double CLIMB_MOTOR_MAX_VELOCITY_RPS = 6000.0,
      CLIMB_MOTOR_MAX_ACCELERATION_RPS_SQUARED = 6000.0;
  public static final double CLIMB_TALONS_SUPPLY_CURRENT_LIMIT_AMPS = 80.0,
      CLIMB_TALONS_STATOR_CURRENT_LIMIT_AMPS = 80.0;

  public static final double CLIMB_MARGIN_DEGREES = 0.5;
  public static final double TEST_CLIMB_MOVEMENT_VOLTAGE = 4.0;

  public static final double CLIMBING_SERVO_UNLOCKED_POSITION_DEGREES = 90.0,
      CLIMBING_SERVO_LOCKED_POSITION_DEGREES = 70.0; // TODO test these values
  // if the lock position isn't 70, then test 110; if not 110, then try arbitrary values
}
