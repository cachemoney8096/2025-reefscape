package frc.robot.subsystems.arm;

import frc.robot.Constants;

public class ArmCal {
  public static final double ARM_POSITION_INTAKE_DEGREES = 0.0,
      ARM_POSITION_HOME_DEGREES = 57.0,
      ARM_POSITION_L1_DEGREES = 99.0,
      ARM_POSITION_L2_DEGREES = 99.0,
      ARM_POSITION_L3_DEGREES = 112.0,
      ARM_POSITION_L4_DEGREES = 150.35,
      ARM_POSITION_DEEP_CLIMB_DEGREES = 26.0;
  public static final double ARM_SUPPLY_CURRENT_LIMIT_AMPS = 80.0,
      ARM_STATOR_CURRENT_LIMIT_AMPS = 80.0;

  public static final double ARM_MARGIN_DEGREES = 0.5;
  public static final double TEST_ARM_MOVEMENT_VOLTAGE = 6.0;

  public static final double ARM_MOTOR_P = 3.5,
      ARM_MOTOR_I = 0.0,
      ARM_MOTOR_D = 1.0,
      ARM_MOTOR_FF = 0.001;

  // TODO if the arm doesn't work, this could be the cause
  public static final double ARM_MOTOR_MAX_VELOCITY_RPS = 1.0,
      ARM_MOTOR_MAX_ACCERLATION_RPS_SQUARED = 2.5;
}
