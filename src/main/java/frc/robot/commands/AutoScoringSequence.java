package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.claw.Claw;

public class AutoScoringSequence extends SequentialCommandGroup {
  public AutoScoringSequence(Claw claw) {
    addRequirements(claw);
    addCommands(
        new InstantCommand(() -> claw.runMotorsScoring()),
        new WaitCommand(3.0),
        new InstantCommand(() -> claw.stopMotors()));
  }
}
