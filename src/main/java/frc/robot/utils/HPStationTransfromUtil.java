package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class HPStationTransfromUtil {

    private final double BASE_X_TRANSFORM_METERS = Units.inchesToMeters(10.6);
    private final double BASE_Y_TRANSFORM_METERS = Units.inchesToMeters(14.6);

    // The left or right HP station
    public enum Station {
        LEFT,
        RIGHT
    }

    // Position on the HP station that the robot is going to
    public enum Position {
        ONE,
        TWO
    }

    public Translation2d getHPStationTransfrom(Station station, Position position, boolean isBlueTeam) {
        double x = BASE_X_TRANSFORM_METERS;
        double y = BASE_Y_TRANSFORM_METERS;

        if (station == Station.RIGHT) {
            y *= -1;
        }

        if (position == Position.TWO) {
            x *= -1;
            y *= -1;
        }

        if (!isBlueTeam) {
            x *= -1;
            y *= -1;
        }

        Translation2d out = new Translation2d(x, y);

        return(out);
    }
    
}
