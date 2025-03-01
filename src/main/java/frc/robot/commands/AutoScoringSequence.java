package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class AutoScoringSequence extends SequentialCommandGroup {
  public AutoScoringSequence(Arm arm, Elevator elevator, Claw claw) {
    addRequirements(arm, elevator, claw);
    addCommands(
        new WaitUntilCommand(
            () -> {
              return arm.atDesiredArmPosition()
                  && elevator.atDesiredPosition();
            }),
        new InstantCommand(() -> claw.runMotorsScoring()),
        new WaitUntilCommand(0.5),
        new InstantCommand(() -> claw.stopMotors()));
  }
}
