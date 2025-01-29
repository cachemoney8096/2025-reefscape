package frc.robot.commands;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AlgeaKnockoffPrepSeqeunce extends SequentialCommandGroup  {
    public AlgeaKnockoffPrepSeqeunce(Elevator elevator, Arm arm, Climb climb, boolean isHigh) {
        addRequirements(elevator, arm, climb);
        addCommands(

            new ConditionalCommand(
                new InstantCommand(() -> climb.setDesiredClimbPosition(ClimbPosition.CLEAR_OF_ARM)),
                new InstantCommand(),
                () -> climb.isClimbInInterferenceZone()),
                new InstantCommand(() -> elevator.setDesiredPosition(isHigh ? ElevatorHeight.SCORE_L2 : ElevatorHeight.SCORE_L1)),
            new WaitUntilCommand(() -> climb.atDesiredPosition()),
            new InstantCommand(() -> arm.setDesiredPosition(isHigh ? ArmPosition.L2 : ArmPosition.L1))
            );

    }       
}