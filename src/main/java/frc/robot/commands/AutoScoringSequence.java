package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;

public class AutoScoringSequence extends SequentialCommandGroup {
  public AutoScoringSequence(Arm arm, Elevator elevator, Claw claw) {
    addRequirements(arm, elevator, claw);
    addCommands(
        new WaitUntilCommand(
            () -> {
              return arm.atDesiredArmPosition() && elevator.atDesiredPosition();
            }).withTimeout(2.0),
        new InstantCommand(() -> claw.runMotorsScoring()),
        new WaitUntilCommand(3.0),
        new InstantCommand(() -> claw.stopMotors()));
  }
}
