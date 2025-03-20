package frc.robot.utils;

import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class PrepStateUtil {
    public enum SCORE_HEIGHT {
        L1,
        L2,
        L3,
        L4,
    }
    public enum INTAKE_CLIMB_LOCATION {
        LEFT,
        CENTER,
        RIGHT
    }
    public enum SCORE_LOCATION {
        LEFT,
        RIGHT
    }

    private SCORE_HEIGHT prepScoreHeight = SCORE_HEIGHT.L2;
    private INTAKE_CLIMB_LOCATION prepIntakeClimbLocation = INTAKE_CLIMB_LOCATION.CENTER;
    private SCORE_LOCATION prepScoreLocation = SCORE_LOCATION.LEFT;

    public void setPrepScoreHeight(SCORE_HEIGHT p) {
        prepScoreHeight = p;
    }
    public void setPrepIntakeClimbLocation(INTAKE_CLIMB_LOCATION p) {
        prepIntakeClimbLocation = p;
    }
    public void setPrepScoreLocation(SCORE_LOCATION p) {
        prepScoreLocation = p;
    }

    public SCORE_HEIGHT getPrepScoreHeight() {
        return this.prepScoreHeight;
    }
    public INTAKE_CLIMB_LOCATION getPrepIntakeClimbLocation() {
        return this.prepIntakeClimbLocation;
    }
    public SCORE_LOCATION getPrepScoreLocation() {
        return this.prepScoreLocation;
    }

    public ElevatorHeight getElevatorHeight(){
        if (prepScoreHeight == PrepStateUtil.SCORE_HEIGHT.L2) {
              return ElevatorHeight.SCORE_L2;
            } else if (prepScoreHeight == PrepStateUtil.SCORE_HEIGHT.L3) {
              return ElevatorHeight.SCORE_L3;
            } else if (prepScoreHeight == PrepStateUtil.SCORE_HEIGHT.L4) {
              return ElevatorHeight.SCORE_L4;
            } else {
              return ElevatorHeight.SCORE_L1;
            }
    }

    public ArmPosition getArmPosition(){
        if (prepScoreHeight == PrepStateUtil.SCORE_HEIGHT.L2) {
              return ArmPosition.L2;
            } else if (prepScoreHeight == PrepStateUtil.SCORE_HEIGHT.L3) {
              return ArmPosition.L3;
            } else if (prepScoreHeight == PrepStateUtil.SCORE_HEIGHT.L4) {
              return ArmPosition.L4;
            } else {
              return ArmPosition.L1;
            }
    }
}