package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ScoringLimelight.ScoringLimelight;
import frc.robot.subsystems.ScoringLimelight.ScoringLimelightConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.utils.ClimbUtil;
import frc.robot.utils.MatchStateUtil;
import frc.robot.utils.PrepStateUtil;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class DeepClimbPrep extends SequentialCommandGroup {
  public static Transform2d robotToTag;
  public static Pose2d targetPose;

  public DeepClimbPrep(
      Climb climb,
      Arm arm,
      ScoringLimelight scoringLimelight,
      PrepStateUtil prepStateUtil,
      MatchStateUtil msu,
      CommandSwerveDrivetrain drive,
      Elevator elevator,
      Lights lights) {
    addRequirements(climb, arm);

    final PrepStateUtil.INTAKE_CLIMB_LOCATION location = prepStateUtil.getPrepIntakeClimbLocation();

    BooleanSupplier checkForTag =
        () -> {
          Optional<Transform2d> robotToTagOptional = scoringLimelight.checkForTag();
          if (robotToTagOptional.isPresent()) {
            robotToTag = robotToTagOptional.get();
            int id =
                (int)
                    NetworkTableInstance.getDefault()
                        .getTable(ScoringLimelightConstants.SCORING_LIMELIGHT_NAME)
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
        new InstantCommand(() -> lights.setLEDColor(LightCode.CLIMB_PREP_DEEP)),
        /* fall back on manual if we don't see a tag */
        new ConditionalCommand(
            new SequentialCommandGroup(
                /* we saw a tag */
                new InstantCommand(
                    () -> {
                      // translate as needed
                      if (location == PrepStateUtil.INTAKE_CLIMB_LOCATION.RIGHT) {
                        robotToTag =
                            robotToTag.plus(
                                new Transform2d(
                                    ClimbUtil.getClimbTransform(
                                        ClimbUtil.CagePosition.RIGHT, msu.isRed()),
                                    new Rotation2d()));
                      } else if (location == PrepStateUtil.INTAKE_CLIMB_LOCATION.LEFT) {
                        robotToTag =
                            robotToTag.plus(
                                new Transform2d(
                                    ClimbUtil.getClimbTransform(
                                        ClimbUtil.CagePosition.LEFT, msu.isRed()),
                                    new Rotation2d()));
                      }
                      // rotate to face the correct way (the rotation here could end up being 180),
                      // these rotations are not technically needed, but increase precision
                      targetPose = drive.getState().Pose.plus(robotToTag);
                      targetPose =
                          new Pose2d(
                              targetPose.getTranslation(),
                              msu.isRed() ? new Rotation2d() : new Rotation2d(180.0));
                    }),
                new InstantCommand(
                    () -> {
                      drive.driveToPose(drive.getState().Pose, targetPose);
                    })),
            new InstantCommand(),
            () -> false),
        deepClimbPrep,
        new InstantCommand(() -> lights.setLEDColor(LightCode.READY_TO_CLIMB)));
  }
}
