package frc.robot.commands;

import frc.robot.subsystems.claw.Claw;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ClawScoringSequence extends SequentialCommandGroup  {
    public ClawScoringSequence(Claw claw) {
        addRequirements(claw);
        addCommands(
            new InstantCommand(() -> claw.runMotorsScoring()),
            new WaitUntilCommand(1.0),
            new InstantCommand(() -> claw.stopMotors())
        );
    }
}