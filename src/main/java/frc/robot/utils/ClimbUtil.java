package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimbUtil {
  private final double LOCATION_OFFSET_Y_METERS =
      Units.inchesToMeters(Constants.CLIMB_CAGE_LOCATION_OFFSET_INCHES);

  public enum CagePosition {
    LEFT,
    RIGHT,
    CENTER
  }

  private CagePosition cagePosition = CagePosition.CENTER;

  public CagePosition getCagePosition() {
    return this.cagePosition;
  }

  public void setCagePosition(CagePosition cagePosition) {
    this.cagePosition = cagePosition;
  }

  public Translation2d getClimbTransform(CagePosition cagePosition, boolean isBlueTeam) {
    double y = LOCATION_OFFSET_Y_METERS;

    if (cagePosition == CagePosition.LEFT) {
      y *= -1;
    } else if (cagePosition == CagePosition.CENTER) {
      y = 0.0;
    }

    if (!isBlueTeam) {
      y *= -1;
    }

    Translation2d out = new Translation2d(0.0, y);

    return (out);
  }
}
