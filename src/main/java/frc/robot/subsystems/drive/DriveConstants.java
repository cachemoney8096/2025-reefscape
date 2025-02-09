package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveConstants {
    public static final double WHEEL_DIAMETER_FUDGE_FACTOR = Constants.PLACEHOLDER_DOUBLE;

    public static final double DRIVE_MOTOR_FREE_SPEED_RPS = Constants.KRAKEN_FREE_SPEED_RPM / 60,
        WHEEL_DIAMETER_METERS = 3 * Units.inchesToMeters(WHEEL_DIAMETER_FUDGE_FACTOR),
        WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    public static final boolean GYRO_REVERSED = Constants.PLACEHOLDER_BOOLEAN;

    public static final double MAX_SPEED_METERS_PER_SECOND = Constants.PLACEHOLDER_DOUBLE;

    public static final double ROBOT_MASS_KG = Constants.PLACEHOLDER_DOUBLE;

    public static final double ROBOT_MOMENT_OF_INERTIA = Constants.PLACEHOLDER_DOUBLE;

    public static final double WHEEL_COEFFICIENT_OF_FRICTION = Constants.PLACEHOLDER_DOUBLE;

    public static final double MOTOR_CURRENT_LIMIT = Constants.PLACEHOLDER_DOUBLE;

    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(Constants.PLACEHOLDER_DOUBLE);

    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(Constants.PLACEHOLDER_DOUBLE);

    public static final double DRIVE_MOTOR_VOLTAGE = Constants.PLACEHOLDER_DOUBLE;

    public static final double DRIVE_MOTOR_STALL_TORQUE_NEWTON_METERS = Constants.PLACEHOLDER_DOUBLE;

    public static final double DRIVE_MOTOR_STALL_CURRENT = Constants.PLACEHOLDER_DOUBLE;

    public static final double DRIVE_MOTOR_FREE_CURRENT = Constants.PLACEHOLDER_DOUBLE;

    public static final double MAX_ANGULAR_SPEED_RAD_PER_SECONDS = Constants.PLACEHOLDER_DOUBLE;

    public static final double DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND = Constants.PLACEHOLDER_DOUBLE;

    public static final double TURN_MODULE_RELATIVE_ENCODER_GEAR_RATIO = Constants.PLACEHOLDER_DOUBLE,
        TURN_MODULE_ABSOLUTE_ENCODER_GEAR_RATIO = Constants.PLACEHOLDER_DOUBLE,
        TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = Constants.PLACEHOLDER_DOUBLE,
        TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS = Constants.PLACEHOLDER_DOUBLE;

    public static final double TURNING_MOTOR_IDLE_MODE = Constants.PLACEHOLDER_DOUBLE,
        TURNING_MOTOR_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE;

    public static final double DRIVING_MOTOR_REDUCTION = Constants.PLACEHOLDER_DOUBLE,
        DRIVING_MOTOR_STATOR_TELEOP_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE,
        DRIVING_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_DOUBLE;
        

    public static final boolean TURNING_ENCODER_INVERTED = Constants.PLACEHOLDER_BOOLEAN;

    public static final PIDConstants PATH_TRANSLATION_CONTROLLER = new PIDConstants(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE),
      PATH_ROTATION_CONTROLLER = new PIDConstants(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);

    public static final PPHolonomicDriveController DRIVE_CONTROLLER = new PPHolonomicDriveController(
        PATH_TRANSLATION_CONTROLLER,
        PATH_ROTATION_CONTROLLER
    );

    public static final Translation2d FRONT_LEFT_MODULE_OFFSET = new Translation2d(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE),
        FRONT_RIGHT_MODULE_OFFSET = new Translation2d(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE),
        REAR_LEFT_MODULE_OFFSET = new Translation2d(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE),
        REAR_RIGHT_MODULE_OFFSET = new Translation2d(Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);

    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
    new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE_METERS / 2, DriveConstants.TRACK_WIDTH_METERS / 2),
        new Translation2d(WHEEL_BASE_METERS / 2, -DriveConstants.TRACK_WIDTH_METERS / 2),
        new Translation2d(-WHEEL_BASE_METERS / 2, DriveConstants.TRACK_WIDTH_METERS / 2),
        new Translation2d(-WHEEL_BASE_METERS / 2, -DriveConstants.TRACK_WIDTH_METERS / 2));

    public static final RobotConfig ROBOT_CONFIG = 
    new RobotConfig(
        DriveConstants.ROBOT_MASS_KG, DriveConstants.ROBOT_MOMENT_OF_INERTIA, new ModuleConfig(
        DriveConstants.WHEEL_DIAMETER_METERS,
        DriveConstants.MAX_SPEED_METERS_PER_SECOND, 
        DriveConstants.WHEEL_COEFFICIENT_OF_FRICTION, 
        new DCMotor(
            DriveConstants.DRIVE_MOTOR_VOLTAGE, 
            DriveConstants.DRIVE_MOTOR_STALL_TORQUE_NEWTON_METERS, 
            DriveConstants.DRIVE_MOTOR_STALL_CURRENT, 
            DriveConstants.DRIVE_MOTOR_FREE_CURRENT, 
            Units.rotationsPerMinuteToRadiansPerSecond(DriveConstants.DRIVE_MOTOR_FREE_SPEED_RPS / 60), 
            4), 
        DriveConstants.MOTOR_CURRENT_LIMIT, 4),
        DriveConstants.FRONT_LEFT_MODULE_OFFSET, 
        DriveConstants.FRONT_RIGHT_MODULE_OFFSET, 
        DriveConstants.REAR_LEFT_MODULE_OFFSET, 
        DriveConstants.REAR_RIGHT_MODULE_OFFSET);
}