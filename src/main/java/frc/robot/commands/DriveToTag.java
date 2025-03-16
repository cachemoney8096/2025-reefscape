package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeLimelight.IntakeLimelight;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class DriveToTag extends SequentialCommandGroup{
    public static Transform2d robotToTag;

    public DriveToTag(CommandSwerveDrivetrain drive, IntakeLimelight limelight){
        BooleanSupplier checkForTag =
        () -> {
          Optional<Transform2d> robotToTagOptional = limelight.checkForTag();
          if (robotToTagOptional.isPresent()) {
            robotToTag = robotToTagOptional.get();
            return true;
          }
          return false;
        };
        addCommands(
            new ConditionalCommand(
                new InstantCommand(()->{Pose2d targetPose = drive.getState().Pose.plus(robotToTag);drive.driveToPose(drive.getState().Pose, targetPose);}), 
                new InstantCommand(()->System.out.println("did not see a tag")), 
                checkForTag)
        );
    }
}
