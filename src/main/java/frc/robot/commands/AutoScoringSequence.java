package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.claw.Claw;

public class AutoScoringSequence extends SequentialCommandGroup {
  public AutoScoringSequence(Claw claw) {
    addRequirements(claw);
    addCommands(
        new InstantCommand(() -> claw.runMotorsScoring()),
        new WaitUntilCommand(0.5),
        new InstantCommand(() -> claw.stopMotors()));
  }
}
