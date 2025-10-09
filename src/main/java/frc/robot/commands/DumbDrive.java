package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DumbDrive extends SequentialCommandGroup{
    public DumbDrive(BiConsumer<Double, Double> rrVelocitySetter, Consumer<Double> headingSetter, double heading, Supplier<Boolean> joystickInput, String llName, double MaxSpeed, CommandSwerveDrivetrain drivetrain){
        Pose3d tagPose3dRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(llName);
        if(tagPose3dRobotSpace.getX() == 0 && tagPose3dRobotSpace.getY() == 0){
            System.out.println("no tag seen");
            return;
        }
        System.out.println("dumbdrive activated");
        double targetHeadingDeg = heading - Math.toDegrees(tagPose3dRobotSpace.getRotation().getY());
        double speedMetersPerSecond = 0.2;
        double distanceOffsetMeters = 0.2;
        double horizontalOffsetMeters = 0.0;
        double xDistMeters = tagPose3dRobotSpace.getZ() + distanceOffsetMeters; //to tag
        double yDistMeters = tagPose3dRobotSpace.getX() + horizontalOffsetMeters; //side to side
        double timeToTagSeconds = xDistMeters / speedMetersPerSecond;
        double timeToAlignSeconds = yDistMeters / speedMetersPerSecond;
        addCommands(
            new InstantCommand(()->{headingSetter.accept(targetHeadingDeg);}),
            new WaitCommand(0.5),
            new InstantCommand(()->{rrVelocitySetter.accept(0.0, speedMetersPerSecond);}),
            new WaitCommand(timeToAlignSeconds),
            new InstantCommand(()->{rrVelocitySetter.accept(speedMetersPerSecond, 0.0);}),
            new WaitCommand(timeToTagSeconds),
            new InstantCommand(()->{rrVelocitySetter.accept(0.0, 0.0);})
        );
    }
}
