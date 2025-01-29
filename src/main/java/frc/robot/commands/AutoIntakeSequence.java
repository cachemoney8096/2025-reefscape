package frc.robot.commands;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoIntakeSequence extends SequentialCommandGroup {
    public AutoIntakeSequence(Elevator elevator, Arm arm, Claw claw) {
        addRequirements(elevator, arm, claw);
        addCommands(
            new InstantCommand(() -> elevator.setDesiredPosition(Elevator.ElevatorHeight.HOME)),
            new InstantCommand(() -> arm.setDesiredPosition(Arm.ArmPosition.INTAKE)),
            new InstantCommand(() -> claw.runMotorsIntaking()),
            new WaitUntilCommand(claw::beamBreakSeesObject),
            new InstantCommand(() -> claw.stopMotors())
        );
    }
}



