package frc.robot.utils;

public class HumanPlayerStationLocationUtil {
    public enum Side {
        RIGHT,
        LEFT
    }
    public enum Position {
        ONE,
        TWO
    }

    private Side side = Side.RIGHT;
    private Position position = Position.ONE;

    public Side getSide() {
        return this.side;
    }

    public void setSide(Side side) {
        this.side = side;
    }

    public Position getPosition() {
        return this.position;
    }
    
    public void setPosition(Position position) {
        this.position = position;
    }
}
