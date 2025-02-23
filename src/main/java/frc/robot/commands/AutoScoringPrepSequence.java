package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class AutoScoringPrepSequence extends SequentialCommandGroup {
  public AutoScoringPrepSequence(Elevator elevator, Arm arm, Claw claw) {
    addRequirements(elevator, arm, claw);
    addCommands(
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.SCORE_L4)),
        new WaitUntilCommand(() -> {return elevator.atElevatorPosition(ElevatorHeight.ARM_CLEAR_OF_CLIMB);}),
        new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.L4)));
  }
}
