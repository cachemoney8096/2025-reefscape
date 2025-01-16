package frc.robot.subsystems.arm;

import frc.robot.Constants;

public class ArmCal {
        public static final double ARM_POSITION_HOME_DEGREES = Constants.PLACEHOLDER_DOUBLE,
                        ARM_POSITION_L2_DEGREES = Constants.PLACEHOLDER_DOUBLE,
                        ARM_POSITION_L3_DEGREES = Constants.PLACEHOLDER_DOUBLE,
                        ARM_POSITION_L4_DEGREES = Constants.PLACEHOLDER_DOUBLE,
                        ARM_POSITION_INTAKE_DEGREES = Constants.PLACEHOLDER_DOUBLE;
        public static final double ARM_SUPPLY_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE,
                        ARM_STATOR_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE;

        public static final double ARM_MARGIN_DEGREES = Constants.PLACEHOLDER_DOUBLE;

        public static final double ARM_MOTOR_P = Constants.PLACEHOLDER_DOUBLE,
                        ARM_MOTOR_I = Constants.PLACEHOLDER_DOUBLE,
                        ARM_MOTOR_D = Constants.PLACEHOLDER_DOUBLE,
                        ARM_MOTOR_FF = Constants.PLACEHOLDER_DOUBLE;
        public static final double ARM_INTERFERENCE_THRESHOLD_MAX_DEGREES = Constants.PLACEHOLDER_DOUBLE, 
                ARM_INTERFERENCE_THRESHOLD_MIN_DEGREES = Constants.PLACEHOLDER_DOUBLE;
        public static final double ARM_MOTOR_MAX_VELOCITY_DPS = Constants.PLACEHOLDER_DOUBLE, ARM_MOTOR_MAX_ACCERLATION_DPS_SQUARED = Constants.PLACEHOLDER_DOUBLE;
}
