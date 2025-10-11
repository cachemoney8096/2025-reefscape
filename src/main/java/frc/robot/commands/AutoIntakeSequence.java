package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.lights.Lights;

public class AutoIntakeSequence extends SequentialCommandGroup {
  public AutoIntakeSequence(Elevator elevator, Arm arm, Claw claw, Lights lights) {
    // addRequirements(elevator, arm, claw);
    // addCommands(
    //     new InstantCommand(() -> elevator.setDesiredPosition(Elevator.ElevatorHeight.INTAKE)),
    //     new InstantCommand(() -> arm.setDesiredPosition(Arm.ArmPosition.INTAKE)),
    //     new WaitUntilCommand(
    //         () -> {
    //           return elevator.atDesiredPosition() && arm.atDesiredArmPosition();
    //         }),
    //     new InstantCommand(() -> claw.runMotorsIntaking()));
  }
}
