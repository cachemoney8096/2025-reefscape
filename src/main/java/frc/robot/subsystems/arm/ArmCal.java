package frc.robot.subsystems.arm;

    public class ArmCal {
  public static final double ARM_POSITION_INTAKE_DEGREES = 106.0 - 52.9, // 141.5 - 57,
      ARM_POSITION_HOME_DEGREES = 106.0, // 108
      ARM_POSITION_L1_DEGREES = 106.0 + 42,
      ARM_POSITION_L2_DEGREES = 106.0 + 42,
      ARM_POSITION_L3_DEGREES = 106.0 + 42 + 13,
      ARM_POSITION_L4_DEGREES = 106.0 + 42 + 13,
      ARM_POSITION_DEEP_CLIMB_DEGREES = 106.0 + 20; // 141.5 + 29;
  public static final double ARM_SUPPLY_CURRENT_LIMIT_AMPS = 80.0,
      ARM_STATOR_CURRENT_LIMIT_AMPS = 80.0;

  public static final double ARM_MARGIN_DEGREES = 7.0;
  public static final double TEST_ARM_MOVEMENT_VOLTAGE = 6.0;

  public static final double ARM_MOTOR_P = 0.8, // 0.75, // should be 2.5 when not having coral
      ARM_MOTOR_I = 0.001,
      ARM_MOTOR_D = 0.0, // should be 0.0 when not having coral
      ARM_MOTOR_FF = 0.0;

  public static final double MOTOR_TO_ARM_ROTATIONS = 90.0; //90 rotations of motor to 1 arm rotation

  public static final double ARM_MOTOR_MAX_VELOCITY_RPS = 6000.0,
      ARM_MOTOR_MAX_ACCERLATION_RPS_SQUARED = 100000.0;
}
