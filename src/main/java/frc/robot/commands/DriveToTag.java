package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ScoringLimelight.ScoringLimelight;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class DriveToTag extends SequentialCommandGroup {
  public static Transform2d robotToTag;
  public static Transform2d rotatedRobotToTag;

  public DriveToTag(CommandSwerveDrivetrain drive, ScoringLimelight limelight) {
    BooleanSupplier checkForTag =
        () -> {
          Optional<Transform2d> robotToTagOptional = limelight.checkForTag();
          if (robotToTagOptional.isPresent()) {
            robotToTag = robotToTagOptional.get();
            rotatedRobotToTag =
                new Transform2d(
                    new Translation2d(
                        -robotToTag.getTranslation().getX(), -robotToTag.getTranslation().getY()),
                    robotToTag.getRotation());
            return true;
          }
          return false;
        };
    addCommands(
        new ConditionalCommand( // offset from bumpers should be 0.406 m
            new InstantCommand(
                () -> {
                  System.out.println(
                      "current pose: "
                          + drive.getState().Pose
                          + "\nrobot to tag : "
                          + robotToTag.toString()
                          + "\n");
                  Pose2d targetPose = drive.getState().Pose.plus(rotatedRobotToTag);
                  System.out.println("target pose: " + targetPose.toString() + "\n");
                  drive.driveToPose(drive.getState().Pose, targetPose);
                }),
            new InstantCommand(() -> System.out.println("did not see a tag")),
            checkForTag));
  }
}
