package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimbUtil {
  private static final double LOCATION_OFFSET_Y_METERS =
      Units.inchesToMeters(Constants.CLIMB_CAGE_LOCATION_OFFSET_INCHES);

  public enum CagePosition {
    LEFT,
    RIGHT,
    CENTER
  }

  public static Translation2d getClimbTransform(CagePosition cagePosition, boolean isRed) {
    double y = LOCATION_OFFSET_Y_METERS;
    if (cagePosition == CagePosition.RIGHT) {
      y *= -1;
    } else if (cagePosition == CagePosition.CENTER) {
      y = 0.0;
    }
    if (isRed) {
      y *= -1;
    }
    return new Translation2d(0.0, y);
  }
}
