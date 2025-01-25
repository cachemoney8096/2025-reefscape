package frc.robot.commands;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AlgeaKnockoffPrepSeqeunce extends SequentialCommandGroup  {
    public AlgeaKnockoffPrepSeqeunce(Elevator elevator, Arm arm, boolean isHigh) {
        addRequirements(elevator);
        addCommands(
            new ConditionalCommand(
                new SequentialCommandGroup(
                new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.SCORE_L2)),
                new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.L2))
            ), new SequentialCommandGroup(
                new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.SCORE_L1)),
                new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.L1))
            ), ()->isHigh)
        );
    }       
}