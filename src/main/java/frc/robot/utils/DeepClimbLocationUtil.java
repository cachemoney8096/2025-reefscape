package frc.robot.utils;

public class DeepClimbLocationUtil {
    public enum Position {
        ONE,
        TWO,
        THREE
    }
    
    private Position position = Position.ONE;

    public Position gePosition() {
        return this.position;
    }

    public void setPosition(Position position) {
        this.position = position;
    }
}
