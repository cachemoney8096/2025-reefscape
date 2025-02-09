package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveConstants {
    public static final double WHEEL_DIAMETER_FUDGE_FACTOR = Constants.PLACEHOLDER_DOUBLE;

    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = Constants.KRAKEN_FREE_SPEED_RPM / 60,
        WHEEL_DIAMETER_METERS = 3 * Units.inchesToMeters(WHEEL_DIAMETER_FUDGE_FACTOR),
        WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    public static final double MAX_SPEED_METERS_PER_SECOND = Constants.PLACEHOLDER_DOUBLE;

    public static final double ROBOT_MASS_KG = Constants.PLACEHOLDER_DOUBLE;

    public static final double ROBOT_MOMENT_OF_INERTIA = Constants.PLACEHOLDER_DOUBLE;

    public static final double WHEEL_COEFFICIENT_OF_FRICTION = Constants.PLACEHOLDER_DOUBLE;

    public static final double MOTOR_CURRENT_LIMIT = Constants.PLACEHOLDER_DOUBLE;

    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(Constants.PLACEHOLDER_DOUBLE);

    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(Constants.PLACEHOLDER_DOUBLE);

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
}