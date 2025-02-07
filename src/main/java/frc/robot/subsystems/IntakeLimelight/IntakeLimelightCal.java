package frc.robot.subsystems.IntakeLimelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeLimelightCal {
  public static final double LIMELIGHT_YAW_DEGREES = 0.0;

  /** offset from AprilTags @ Human Player Station */
  public static final Transform2d HPS_OFFSET =
      new Transform2d(
          new Translation2d(
              -Units.inchesToMeters(Constants.PLACEHOLDER_DOUBLE), Constants.PLACEHOLDER_DOUBLE),
          new Rotation2d(Constants.PLACEHOLDER_DOUBLE));

  public static final double LIMELIGHT_DETECTION_OFFSET_DEGREES = 0.0;
}
