package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ScoringLimelight.ScoringLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.utils.ClimbUtil;
import frc.robot.utils.MatchStateUtil;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class DeepClimbPrep extends SequentialCommandGroup {
  public static Transform2d robotToTag;
  public static Pose2d targetPose;

  public DeepClimbPrep(
      Climb climb,
      Arm arm,
      ScoringLimelight scoringLimelight,
      RobotContainer.IntakeClimbLocation location,
      MatchStateUtil msu,
      DriveSubsystem drive,
      Elevator elevator) {
    addRequirements(climb, arm);

    BooleanSupplier checkForTag =
        () -> {
          Optional<Transform2d> robotToTagOptional = scoringLimelight.checkForTag();
          if (robotToTagOptional.isPresent()) {
            robotToTag = robotToTagOptional.get();
            int id =
                (int)
                    NetworkTableInstance.getDefault()
                        .getTable("limelight-scoring")
                        .getEntry("tid")
                        .getDouble(0.0);
            return (id == 14 && !msu.isRed()) || (id == 5 && msu.isRed());
          }
          return false;
        };
    SequentialCommandGroup deepClimbPrep =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> elevator.setDesiredPosition(ElevatorHeight.ARM_CLEAR_OF_CLIMB)),
            new WaitUntilCommand(elevator::armMovementAllowed),
            new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.DEEP_CLIMB)),
            new WaitUntilCommand(arm::atDesiredArmPosition),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)),
            new InstantCommand(() -> climb.setDesiredClimbPosition(ClimbPosition.CLIMBING_PREP)));

    addCommands(
        /* fall back on manual if we don't see a tag */
        new ConditionalCommand(
            new SequentialCommandGroup(
                /* we saw a tag */
                new InstantCommand(
                    () -> {
                      // translate as needed
                      if (location == RobotContainer.IntakeClimbLocation.RIGHT) {
                        robotToTag =
                            robotToTag.plus(
                                new Transform2d(
                                    ClimbUtil.getClimbTransform(
                                        ClimbUtil.CagePosition.RIGHT, msu.isRed()),
                                    new Rotation2d()));
                      } else if (location == RobotContainer.IntakeClimbLocation.LEFT) {
                        robotToTag =
                            robotToTag.plus(
                                new Transform2d(
                                    ClimbUtil.getClimbTransform(
                                        ClimbUtil.CagePosition.LEFT, msu.isRed()),
                                    new Rotation2d()));
                      }
                      // rotate to face the correct way (the rotation here could end up being 180),
                      // these rotations are not technically needed, but increase precision
                      targetPose = drive.getPose().plus(robotToTag);
                      targetPose =
                          new Pose2d(
                              targetPose.getTranslation(),
                              msu.isRed() ? new Rotation2d() : new Rotation2d(180.0));
                    }),
                new InstantCommand(() -> drive.driveToPoint(targetPose))),
            // TODO the drive logic here probably won't be the same
            new InstantCommand(),
            checkForTag),
        deepClimbPrep);
  }
}
