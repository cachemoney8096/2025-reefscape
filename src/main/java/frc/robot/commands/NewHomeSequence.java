package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

/** Exactly the HOME sequence you had in RobotContainer, moved into a command. */
public class NewHomeSequence extends SequentialCommandGroup {

  public NewHomeSequence(Arm arm, Elevator elevator, Claw claw) {
    addCommands(
        new InstantCommand(
            () -> {
              elevator.setDesiredPosition(ElevatorHeight.ARM_CLEAR_OF_CLIMB);
              claw.stopMotors();
            }),
        new WaitUntilCommand(elevator::atDesiredPosition),
        new InstantCommand(
            () -> {
              arm.setDesiredPosition(ArmPosition.HOME);
            }),
        new WaitUntilCommand(arm::atDesiredArmPosition),
        new InstantCommand(
            () -> {
              elevator.setDesiredPosition(ElevatorHeight.HOME);
            }));
  }
}
