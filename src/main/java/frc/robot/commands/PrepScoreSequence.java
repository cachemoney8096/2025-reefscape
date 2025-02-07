package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ScoringLimelight.ScoringLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class PrepScoreSequence extends SequentialCommandGroup {
  public PrepScoreSequence(
      Arm arm,
      Elevator elevator,
      ScoringLimelight scoringLimelight,
      ArmPosition armPosition,
      boolean isRight,
      Climb climb) {
    addRequirements(arm, elevator, scoringLimelight);
    boolean isTagValid =
        ScoringLimelight.validScoringTags(
            (NetworkTableInstance.getDefault()
                .getTable("limelight-scoring")
                .getEntry("tid")
                .getDouble(0)));
    addCommands(
        /** Set Desired Arm Pos Set Desired Elevator Pos */
        new InstantCommand(() -> climb.setDesiredClimbPosition(ClimbPosition.CLEAR_OF_ARM)),
        new WaitUntilCommand(climb::atDesiredPosition),
        new ParallelCommandGroup(
            new InstantCommand(
                () ->
                    elevator.setDesiredPosition(
                        ElevatorHeight.values()[Constants.PLACEHOLDER_INT])),
            new InstantCommand(
                /** Need to ensure that Deep Climb is NOT in interference zone */
                () -> arm.setDesiredPosition(ArmPosition.values()[Constants.PLACEHOLDER_INT]))),
        /**
         * If tag is seen, use command sequence based on limelight Otherwise, assume manual drive
         */
        new ConditionalCommand(
            new SequentialCommandGroup(
                /** Get tag and adjust position for offset. This needs drive code. */
                ),
            new SequentialCommandGroup(
                /** Let the driver manually adjust to the offset. This also needs drive code. */
                ),
            () -> (scoringLimelight.checkForTag().isPresent() && isTagValid)));
  }
}
