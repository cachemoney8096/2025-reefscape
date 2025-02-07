package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeLimelight.IntakeLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.ClimbUtil;
import frc.robot.utils.MatchStateUtil;

public class DeepClimbPrep extends SequentialCommandGroup {
  public static Transform2d robotToTag;
  public static Pose2d targetPose;

  public DeepClimbPrep(Climb climb, Arm arm, IntakeLimelight intakeLimelight, RobotContainer.Location location,
      MatchStateUtil msu, DriveSubsystem drive) {
    addRequirements(climb, arm);
    BooleanSupplier checkForTag = () -> {
      Optional<Transform2d> robotToTagOptional = intakeLimelight.checkForTag();
      if (robotToTagOptional.isPresent()) {
        robotToTag = robotToTagOptional.get();
        return true;
      }
      return false;
    };
    SequentialCommandGroup deepClimbPrep = new SequentialCommandGroup(
        /* interferance zones */
        new ConditionalCommand(
            new InstantCommand(),
            new SequentialCommandGroup(
                new InstantCommand(()->arm.setDesiredPosition(ArmPosition.HOME)),
                new WaitUntilCommand(() -> arm.atDesiredArmPosition())),
            () -> {
              return !arm.isArmInInterferenceZone();
            }),
        /* now we can definitely move the climb */
        new InstantCommand(() -> climb.setDesiredClimbPosition(ClimbPosition.CLIMBING_PREP)),
        /* wait until it gets there to set PID for climbing */
        new WaitUntilCommand(() -> climb.atDesiredPosition()),
        new InstantCommand(() -> climb.setClimbingPID()));

    addCommands(
        /* fall back on manual if we don't see a tag */
        new ConditionalCommand(
            new SequentialCommandGroup(
                /* we saw a tag */
                new InstantCommand(() -> {
                  // translate as needed
                  if (location == RobotContainer.Location.RIGHT) {
                    robotToTag = robotToTag.plus(new Transform2d(
                        ClimbUtil.getClimbTransform(ClimbUtil.CagePosition.RIGHT, msu.isRed()), new Rotation2d()));
                  } else if (location == RobotContainer.Location.LEFT) {
                    robotToTag = robotToTag.plus(new Transform2d(
                        ClimbUtil.getClimbTransform(ClimbUtil.CagePosition.LEFT, msu.isRed()), new Rotation2d()));
                  }
                  // rotate to face the correct way (the rotation here could end up being 180)
                  robotToTag = new Transform2d(robotToTag.getTranslation(), new Rotation2d());
                  targetPose = drive.getRobotPose().plus(robotToTag);
                }),
                new InstantCommand(() -> drive.driveToPoint(targetPose)), // TODO the drive logic here probably won't be the same
                deepClimbPrep),
            deepClimbPrep,
            checkForTag));
  }
}
