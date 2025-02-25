package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class DeepClimbScoringSequence extends SequentialCommandGroup {
  public DeepClimbScoringSequence(Arm arm, Climb climb, Elevator elevator) {
    addRequirements(climb, arm, elevator);
    addCommands(
        new WaitUntilCommand(() -> elevator.atElevatorPosition(ElevatorHeight.HOME)),
        new WaitUntilCommand(() -> arm.atArmPosition(ArmPosition.DEEP_CLIMB)),
        new WaitUntilCommand(() -> climb.atClimbPosition(ClimbPosition.CLIMBING_PREP)),
        new InstantCommand(() -> climb.setClimbingPID()),
        new InstantCommand(() -> climb.setDesiredClimbPosition(ClimbPosition.CLIMBING))
    );
  }
}







