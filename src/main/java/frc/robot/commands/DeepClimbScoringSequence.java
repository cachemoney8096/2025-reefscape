package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;

public class DeepClimbScoringSequence extends SequentialCommandGroup {
  public DeepClimbScoringSequence(Climb climb) {
    addRequirements(climb);
    addCommands(
        new WaitUntilCommand(() -> climb.atClimbPosition(ClimbPosition.CLIMBING_PREP)),
        new InstantCommand(() -> climb.setClimbingPID()),
        new InstantCommand(
            () ->
                climb.setDesiredClimbPosition(
                    ClimbPosition.CLIMBING))); // TODO might have to drive at same time
  }
}
