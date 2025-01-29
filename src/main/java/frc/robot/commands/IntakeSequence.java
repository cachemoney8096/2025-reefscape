package frc.robot.commands;

import java.util.Optional;
import java.util.concurrent.locks.Condition;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeLimelight.IntakeLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class IntakeSequence extends SequentialCommandGroup {
    private Optional<Transform2d> isTag = Optional.empty();
    private double tagid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    public IntakeSequence(Claw claw, IntakeLimelight intakeLime, Arm arm, Elevator elevator, Climb climb) {
        isTag = intakeLime.checkForTag();
        SequentialCommandGroup moveArmElevatorClaw = new SequentialCommandGroup(
            /*TODO do drive logic here*/
            // the next two need to be done at the same time as the driving
            new ConditionalCommand(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> climb.setDesiredClimbPosition(ClimbPosition.CLEAR_OF_ARM)),
                        new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.INTAKE))
                    ),
                    new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.INTAKE)), 
                    () -> {return arm.isArmMoveable();}),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)),
            new InstantCommand(() -> claw.runMotorsIntaking()),
            new WaitUntilCommand(claw::beamBreakSeesObject),
            new InstantCommand(() -> claw.stopMotors())
        );
        addRequirements(claw);
        addCommands(
            new ConditionalCommand(
                                    new SequentialCommandGroup(
                                        /*TODO do drive logic here*/
                                        // the next two need to be done at the same time as the driving
                                        moveArmElevatorClaw
                                    ), 
                                    moveArmElevatorClaw, // return out of the command if the robot does not see the intake april tag 
                                    () -> {return (!isTag.isEmpty() && ((tagid == 1) || (tagid == 2) || (tagid == 12) || (tagid == 13)));}
                                )
        );
    }
}
