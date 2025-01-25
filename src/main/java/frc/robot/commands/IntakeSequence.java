package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeLimelight.IntakeLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.Optional;

public class IntakeSequence extends SequentialCommandGroup {
    public IntakeSequence(Claw claw, IntakeLimelight light, Arm arm, Elevator elevator) {
        addRequirements(claw);
        addCommands(
            new ConditionalCommand(/*TODO do drive logic here*/ null, 
                                    new InstantCommand(), // return out of the command if the robot does not see the intake april tag 
                                    () -> {if(light.checkForTag().isEmpty()){return false;} else{return true;}}),
            // the next two need to be done at the same time as the driving
            new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.INTAKE)),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)),
            new InstantCommand(() -> claw.runMotorsIntaking()),
            new WaitUntilCommand(claw::beamBreakSeesObject),
            new InstantCommand(() -> claw.stopMotors())
        );
    }
}
