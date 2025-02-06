package frc.robot.commands;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoScoringSequence extends SequentialCommandGroup {
    public AutoScoringSequence(Elevator elevator, Arm arm, Claw claw) {
        addRequirements(elevator, arm, claw);
        addCommands(
            new InstantCommand(() -> elevator.setDesiredPosition(Elevator.ElevatorHeight.SCORE_L4)),
            new InstantCommand(() -> arm.setDesiredPosition(Arm.ArmPosition.L4)),
            
            new WaitUntilCommand(elevator::atDesiredPosition),
            new WaitUntilCommand(arm::atDesiredArmPosition),

            new InstantCommand(() -> claw.runMotorsScoring()),
            new WaitUntilCommand(0.5),
            new InstantCommand(() -> claw.stopMotors())
        );
    }
}
