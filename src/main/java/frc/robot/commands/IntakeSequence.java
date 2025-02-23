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
import frc.robot.subsystems.IntakeLimelight.IntakeLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.utils.HPUtil;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class IntakeSequence extends SequentialCommandGroup {
  public static Transform2d robotToTag;
  public static Pose2d targetPose;

  public IntakeSequence(
      Claw claw,
      IntakeLimelight intakeLimelight,
      Arm arm,
      Elevator elevator,
      Climb climb,
      RobotContainer.IntakeClimbLocation location,
      DriveSubsystem drive) {
    /* mechanical intake sequence - slightly modified version of what ruben had previously */
    SequentialCommandGroup moveArmElevatorClaw =
        new SequentialCommandGroup(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(
                        () -> climb.setDesiredClimbPosition(ClimbPosition.CLEAR_OF_ARM)),
                    new WaitUntilCommand(() -> climb.atDesiredPosition())),
                new InstantCommand(),
                () -> !arm.isArmInInterferenceZone()),
            new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.INTAKE)),
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)),
            new WaitUntilCommand(
                () -> {
                  return elevator.atDesiredPosition() && arm.atDesiredArmPosition();
                }));

    BooleanSupplier checkForTag =
        () -> {
          Optional<Transform2d> robotToTagOptional = intakeLimelight.checkForTag();
          if (robotToTagOptional.isPresent()) {
            robotToTag = robotToTagOptional.get();
            return true;
          }
          return false;
        };

    addRequirements(claw, arm, elevator, climb);
    /* revert to manual control if we don't see a tag */
    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                /* we saw a tag */
                new InstantCommand(
                    () -> {
                      // translate as needed
                      int id =
                          (int)
                              NetworkTableInstance.getDefault()
                                  .getTable("limelight-intake")
                                  .getEntry("tid")
                                  .getDouble(0.0);
                      HPUtil.Position pos =
                          location == RobotContainer.IntakeClimbLocation.LEFT
                              ? HPUtil.Position.LEFT
                              : HPUtil.Position.RIGHT;
                      switch (id) {
                        case 13: // BL
                          robotToTag =
                              robotToTag.plus(
                                  new Transform2d(
                                      HPUtil.getTranslation(HPUtil.Station.LEFT, pos, false),
                                      new Rotation2d()));
                          break;

                        case 12: // BR
                          robotToTag =
                              robotToTag.plus(
                                  new Transform2d(
                                      HPUtil.getTranslation(HPUtil.Station.RIGHT, pos, false),
                                      new Rotation2d()));
                          break;

                        case 1: // RL
                          robotToTag =
                              robotToTag.plus(
                                  new Transform2d(
                                      HPUtil.getTranslation(HPUtil.Station.LEFT, pos, true),
                                      new Rotation2d()));
                          break;

                        case 2: // RR
                          robotToTag =
                              robotToTag.plus(
                                  new Transform2d(
                                      HPUtil.getTranslation(HPUtil.Station.RIGHT, pos, true),
                                      new Rotation2d()));
                          break;
                      }
                      targetPose = drive.getPose().plus(robotToTag);
                    }),
                new InstantCommand(() -> drive.driveToPoint(targetPose))),
            new InstantCommand(),
            checkForTag),
        moveArmElevatorClaw);
  }
}
