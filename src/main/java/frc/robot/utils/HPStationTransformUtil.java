package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class HPStationTransformUtil {

    private final double LOCATION_OFFSET_X_METERS = Units.inchesToMeters(Constants.HP_INTAKE_TAG_LOCATION_OFFSET_X_INCHES);
    private final double LOCATION_OFFSET_Y_METERS = Units.inchesToMeters(Constants.HP_INTAKE_TAG_LOCATION_OFFSET_Y_INCHES);

    // The left or right HP station (from driver's perspective). 
    public enum Station {
        LEFT,
        RIGHT
    }

    // Position on the HP station that the robot is going to (from robot's perspetive).
    public enum Position {
        LEFT,
        RIGHT,
        TAG
    }

    private Station side = Station.RIGHT;
    private Position position = Position.LEFT;

    public Station getSide() {
        return this.side;
    }

    public void setSide(Station side) {
        this.side = side;
    }

    public Position getPosition() {
        return this.position;
    }

    public void setPosition(Position position) {
        this.position = position;
    }

    public Translation2d getHPStationTransfrom(Station station, Position position, boolean isBlueTeam) {
        double x = LOCATION_OFFSET_X_METERS;
        double y = LOCATION_OFFSET_Y_METERS;

        if (position == Position.TAG) {
            x = 0;
            y = 0;
        }

        if (station == Station.RIGHT) {
            y *= -1;
        }

        if (position == Position.RIGHT) {
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
