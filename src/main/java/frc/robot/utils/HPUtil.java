package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.HashMap;

public class HPUtil {
  private static final double OFFSET_X_METERS =
      Units.inchesToMeters(Constants.HP_INTAKE_TAG_LOCATION_OFFSET_X_INCHES);
  private static final double OFFSET_Y_METERS =
      Units.inchesToMeters(Constants.HP_INTAKE_TAG_LOCATION_OFFSET_Y_INCHES);

  public enum Station {
    LEFT,
    RIGHT
  }

  public enum Position {
    LEFT,
    RIGHT,
    CENTER
  }

  private static HashMap<Pair<Station, Position>, Pair<Double, Double>> blueMap =
      new HashMap<Pair<Station, Position>, Pair<Double, Double>>();

  public static Translation2d getTranslation(Station station, Position position, boolean isRed) {
    blueMap.put(
        new Pair<Station, Position>(Station.LEFT, Position.LEFT),
        new Pair<Double, Double>(-1.0, -1.0));
    blueMap.put(
        new Pair<Station, Position>(Station.LEFT, Position.RIGHT),
        new Pair<Double, Double>(1.0, 1.0));
    blueMap.put(
        new Pair<Station, Position>(Station.RIGHT, Position.LEFT),
        new Pair<Double, Double>(1.0, -1.0));
    blueMap.put(
        new Pair<Station, Position>(Station.RIGHT, Position.RIGHT),
        new Pair<Double, Double>(-1.0, 1.0));
    blueMap.put(
        new Pair<Station, Position>(Station.RIGHT, Position.CENTER),
        new Pair<Double, Double>(0.0, 0.0));
    blueMap.put(
        new Pair<Station, Position>(Station.LEFT, Position.CENTER),
        new Pair<Double, Double>(0.0, 0.0));
    int flipForRed = isRed ? -1 : 1;
    Pair<Double, Double> multipliers = blueMap.get(new Pair<Station, Position>(station, position));
    double xMult = multipliers.getFirst() * flipForRed;
    double yMult = multipliers.getSecond() * flipForRed;
    return new Translation2d(OFFSET_X_METERS * xMult, OFFSET_Y_METERS * yMult);
  }
}
