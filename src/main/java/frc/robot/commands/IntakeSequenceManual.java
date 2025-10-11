package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import java.util.function.Consumer;

public class IntakeSequenceManual extends SequentialCommandGroup {
  public enum Location {
    LEFT,
    RIGHT
  }

  public IntakeSequenceManual(
      Arm arm, Elevator elevator, Claw claw, Location location, Consumer<Double> headingSetter) {

    // PLACEHOLDERS
    final double headingLeft = -15.0;
    final double headingRight = 15.0;

    addCommands(
        new InstantCommand(
            () -> {

              /*if (location == Location.LEFT) {
                headingSetter.accept(headingLeft);
              } else {
                headingSetter.accept(headingRight);
              }*/

              arm.setDesiredPosition(ArmPosition.INTAKE);
              elevator.setDesiredPosition(ElevatorHeight.INTAKE);
              claw.runMotorsIntaking();
            }),
        new WaitUntilCommand(claw::beamBreakSeesObject),
        new InstantCommand(claw::stopMotors));
  }
}
