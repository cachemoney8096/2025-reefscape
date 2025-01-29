package frc.robot.utils;

public class DeepClimbLocationUtil {
    public enum Position {
        ONE,
        TWO,
        THREE
    }
    
    private Position position = Position.ONE;

    public Position getPosition() {
        return this.position;
    }

    public void setPosition(Position position) {
        this.position = position;
    }
}
