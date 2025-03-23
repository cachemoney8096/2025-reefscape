package frc.robot.subsystems.drive.Old;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class BackUpDriveCal {
  public static final int SPARK_INIT_RETRY_ATTEMPTS = Constants.PLACEHOLDER_INT;

  /** Input meters/second, output [-1,1] */
  public static final double DRIVING_P = Constants.PLACEHOLDER_DOUBLE,
      DRIVING_I = Constants.PLACEHOLDER_DOUBLE,
      DRIVING_D = Constants.PLACEHOLDER_DOUBLE,
      DRIVING_FF =
          Constants.PLACEHOLDER_DOUBLE / DriveConstants.DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;

  /** Input radians, output [-1,1] */
  public static final double TURNING_P = Constants.PLACEHOLDER_DOUBLE,
      TURNING_I = Constants.PLACEHOLDER_DOUBLE,
      TURNING_D = Constants.PLACEHOLDER_DOUBLE,
      TURNING_FF = Constants.PLACEHOLDER_DOUBLE;

  /**
   * Angular offset of the modules relative to the zeroing fixture in radians. Ideally should be
   * relative to the fixture but they are actually slightly different.
   */
  public static double
      SWERVE_FRONT_LEFT_ANGULAR_OFFSET_RAD = Constants.PLACEHOLDER_DOUBLE + Math.PI,
      SWERVE_FRONT_RIGHT_ANGULAR_OFFSET_RAD = Constants.PLACEHOLDER_DOUBLE,
      SWERVE_BACK_LEFT_ANGULAR_OFFSET_RAD = Constants.PLACEHOLDER_DOUBLE,
      SWERVE_BACK_RIGHT_ANGULAR_OFFSET_RAD = Constants.PLACEHOLDER_DOUBLE + Math.PI;

  /**
   * Angular offsets of the modules relative to the chassis in radians. The modules form an O when
   * fixtured, so they are iteratively 90 deg from each other.
   */
  public static final double
      FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD =
          SWERVE_FRONT_LEFT_ANGULAR_OFFSET_RAD
              - (Constants.PLACEHOLDER_DOUBLE * Math.PI / Constants.PLACEHOLDER_DOUBLE),
      FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD =
          SWERVE_FRONT_RIGHT_ANGULAR_OFFSET_RAD
              + (Constants.PLACEHOLDER_DOUBLE * Math.PI / Constants.PLACEHOLDER_DOUBLE),
      BACK_LEFT_CHASSIS_ANGULAR_OFFSET_RAD =
          SWERVE_BACK_LEFT_ANGULAR_OFFSET_RAD
              - (Constants.PLACEHOLDER_DOUBLE * Math.PI / Constants.PLACEHOLDER_DOUBLE),
      REAR_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD =
          SWERVE_BACK_RIGHT_ANGULAR_OFFSET_RAD
              + (Constants.PLACEHOLDER_DOUBLE * Math.PI / Constants.PLACEHOLDER_DOUBLE);

  /** Controller on module speed for rotating to target, input degrees [-180,180], output [0,1]. */
  public static final PIDController ROTATE_TO_TARGET_PID_CONTROLLER =
      new PIDController(
          Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);

  /**
   * The value for the keepHeading interpolation table based on normalized translation velocity when
   * the robot is motionless (in xy). We want the lowest output to be 0.013 (that is tuned),
   */
  public static final double MIN_ROTATE_TO_TARGET_PID_OUTPUT =
      Constants.PLACEHOLDER_DOUBLE / ROTATE_TO_TARGET_PID_CONTROLLER.getP();

  /** Feed forward for rotating to target, gets added to or subtracted from PID controller. */
  public static final double ROTATE_TO_TARGET_FF = Constants.PLACEHOLDER_DOUBLE;

  /** If the desired chassis rotation is below this value in [0,1], it is ignored */
  public static final double ROTATION_DEADBAND_THRESHOLD = Constants.PLACEHOLDER_DOUBLE;

  public static final double ROTATION_DEADBAND_THRESHOLD_DEG = Constants.PLACEHOLDER_DOUBLE;

  /** path finding controller for translation and rotation; used in PathPlanner */
  public static final PIDConstants
      PATH_TRANSLATION_CONTROLLER =
          new PIDConstants(
              Constants.PLACEHOLDER_DOUBLE,
              Constants.PLACEHOLDER_DOUBLE,
              Constants.PLACEHOLDER_DOUBLE),
      PATH_ROTATION_CONTROLLER =
          new PIDConstants(
              Constants.PLACEHOLDER_DOUBLE,
              Constants.PLACEHOLDER_DOUBLE,
              Constants.PLACEHOLDER_DOUBLE);

  public static final PPHolonomicDriveController DRIVE_CONTROLLER =
      new PPHolonomicDriveController(PATH_TRANSLATION_CONTROLLER, PATH_ROTATION_CONTROLLER);

  public static final double MEDIUM_LINEAR_SPEED_METERS_PER_SEC = Constants.PLACEHOLDER_DOUBLE,
      MEDIUM_LINEAR_ACCELERATION_METERS_PER_SEC_SQ = Constants.PLACEHOLDER_DOUBLE,
      MEDIUM_ANGULAR_SPEED_RAD_PER_SEC = Math.PI,
      MEDIUM_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = Math.PI;

  public static final double TURNING_ENCODER_ZEROING_THRESHOLD_RAD =
      Units.degreesToRadians(Constants.PLACEHOLDER_DOUBLE);
}
