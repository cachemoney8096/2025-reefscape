package frc.robot.utils;

import edu.wpi.first.math.Pair;
import java.util.HashMap;

public class Testing {
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

  public static void main(String[] args) {
    blueMap.put(
        new Pair<Station, Position>(Station.LEFT, Position.LEFT),
        new Pair<Double, Double>(-1.0, -1.0));
  }
}
