package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;

public class AutoIntakeSequence extends SequentialCommandGroup {
  public AutoIntakeSequence(Elevator elevator, Arm arm, Claw claw) {
    addRequirements(elevator, arm, claw);
    addCommands(
        new InstantCommand(() -> elevator.setDesiredPosition(Elevator.ElevatorHeight.INTAKE)),
        new WaitUntilCommand(elevator::atDesiredPosition),
        new InstantCommand(() -> arm.setDesiredPosition(Arm.ArmPosition.INTAKE)),
        new WaitUntilCommand(arm::atDesiredArmPosition),
        new InstantCommand(() -> claw.runMotorsIntaking()),
        new WaitUntilCommand(claw::beamBreakSeesObject).withTimeout(3.0),
        new InstantCommand(() -> claw.stopMotors()));
  }
}
