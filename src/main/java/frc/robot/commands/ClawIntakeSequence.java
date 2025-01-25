package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.claw.Claw;

public class ClawIntakeSequence extends SequentialCommandGroup {
  public ClawIntakeSequence(Claw claw) {
    addRequirements(claw);
    addCommands(
        new InstantCommand(() -> claw.runMotorsIntaking()),
        new WaitUntilCommand(claw::beamBreakSeesObject),
        new InstantCommand(() -> claw.stopMotors()));
  }
}
