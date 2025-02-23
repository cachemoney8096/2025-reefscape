package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class FinishScore extends SequentialCommandGroup {
  public FinishScore(Claw claw, Elevator elevator, Arm arm, ElevatorHeight height) {
    final ArmPosition p;
    if (height == ElevatorHeight.SCORE_L2) {
      p = ArmPosition.L2;
    } else if (height == ElevatorHeight.SCORE_L3) {
      p = ArmPosition.L3;
    } else if (height == ElevatorHeight.SCORE_L4) {
      p = ArmPosition.L4;
    } else {
      p = ArmPosition.L1;
    }
    addCommands(
        new WaitUntilCommand(
            () -> {
              return elevator.atElevatorPosition(height) && arm.atArmPosition(p);
            }),
        new InstantCommand(
            () -> claw.runMotorsScoring()), // TODO might have to change for scoring L4
        new WaitCommand(1.0),
        new InstantCommand(() -> claw.stopMotors()));
  }
}
