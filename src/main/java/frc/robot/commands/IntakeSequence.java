package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeLimelight.IntakeLimelight;
import frc.robot.subsystems.IntakeLimelight.IntakeLimelightConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.utils.HPUtil;
import frc.robot.utils.PrepStateUtil;

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
      PrepStateUtil prepStateUtil,
      CommandSwerveDrivetrain drive,
      Lights lights) {
    /* mechanical intake sequence */
      final PrepStateUtil.INTAKE_CLIMB_LOCATION location = prepStateUtil.getPrepIntakeClimbLocation();

    SequentialCommandGroup moveArmElevatorClaw =
        new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.INTAKE)),
            new InstantCommand(() -> System.out.println("elevator desired set to intake")),
            new WaitUntilCommand(elevator::armMovementAllowed),
            new InstantCommand(() -> System.out.println("arm movement allowed")),
            new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.INTAKE)),
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
      moveArmElevatorClaw,
        new ConditionalCommand(
            new SequentialCommandGroup(
                /* we saw a tag */
                new InstantCommand(
                    () -> {
                      // translate as needed
                      int id =
                          (int)
                              NetworkTableInstance.getDefault()
                                  .getTable(IntakeLimelightConstants.INTAKE_LIMELIGHT_NAME)
                                  .getEntry("tid")
                                  .getDouble(0.0);
                      HPUtil.Position pos =
                          location == PrepStateUtil.INTAKE_CLIMB_LOCATION.LEFT
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
                      targetPose = drive.getState().Pose.plus(robotToTag);
                    }),
                new InstantCommand(
                    () -> {
                      drive.driveToPose(drive.getState().Pose, targetPose);
                    })),
            new InstantCommand(),
            ()->false));
  }
}
