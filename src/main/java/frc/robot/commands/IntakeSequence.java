package frc.robot.commands;

import java.util.Optional;
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
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class IntakeSequence extends SequentialCommandGroup {
    private Optional<Transform2d> isTag = Optional.empty();
    public IntakeSequence(Claw claw, IntakeLimelight intakeLime, Arm arm, Elevator elevator) {
        isTag = intakeLime.checkForTag();
        addRequirements(claw);
        addCommands(
            new ConditionalCommand(
                                    new SequentialCommandGroup(
                                        /*TODO do drive logic here*/
                                        // the next two need to be done at the same time as the driving
                                        new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.INTAKE)),
                                        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)),
                                        new InstantCommand(() -> claw.runMotorsIntaking()),
                                        new WaitUntilCommand(claw::beamBreakSeesObject),
                                        new InstantCommand(() -> claw.stopMotors())
                                    ), 
                                    new InstantCommand(), // return out of the command if the robot does not see the intake april tag 
                                    () -> {return (!isTag.isEmpty() && (
                                        (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0) == 1) ||
                                        (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0) == 2) ||
                                        (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0) == 12) ||
                                        (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0) == 13)));}
                                )
        );
    }
}
