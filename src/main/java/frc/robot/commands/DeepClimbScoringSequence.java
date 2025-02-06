package frc.robot.commands;

import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DeepClimbScoringSequence extends SequentialCommandGroup  {
    public DeepClimbScoringSequence(Climb climb) {
        addRequirements(climb);
        addCommands(
            new InstantCommand(() -> climb.setClimbingPID()),
            new InstantCommand(() -> climb.setDesiredClimbPosition(ClimbPosition.CLIMBING))

        );
    }       
}