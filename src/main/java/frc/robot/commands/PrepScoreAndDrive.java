package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PrepScoreAndDrive extends ParallelCommandGroup {
  public enum Location {
    LEFT,
    RIGHT
  }

  private double visionBasedX = 1.0; // arbitrary non-zero value
  private double visionBasedY = 1.0;

  public PrepScoreAndDrive(
      Elevator elevator,
      Arm arm,
      ElevatorHeight elevatorHeight,
      Consumer<Pair<Double, Double>> velocitySetter,
      Consumer<Double> headingSetter,
      double heading,
      Supplier<Boolean> joystickInput,
      String llName,
      double MaxSpeed,
      CommandSwerveDrivetrain drivetrain,
      Location location) {
    ProfiledPIDController xPid =
        new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(MaxSpeed * 0.1, MaxSpeed));
    ProfiledPIDController yPid =
        new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(MaxSpeed * 0.1, MaxSpeed));
    SequentialCommandGroup driveToTag =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  Pose3d tagOffset =
                      LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME);
                  Rotation3d tagRot = tagOffset.getRotation();
                  headingSetter.accept(heading - Math.toDegrees(tagRot.getY()));
                }),
            new RepeatCommand(
                    new InstantCommand(
                        () -> {
                          Pose3d tagOffset =
                              LimelightHelpers.getTargetPose3d_RobotSpace(
                                  Constants.LIMELIGHT_FRONT_NAME);
                          if (tagOffset.getX() == 0.0
                              && tagOffset.getY() == 0.0
                              && tagOffset.getZ() == 0.0) {
                            this.cancel();
                          }
                          double xError = tagOffset.getX() + 0.23;
                          double zError = tagOffset.getZ() + 0.3; // 0.2 from tag
                          double robotCXPid =
                              -xPid.calculate(zError, 0); // calc the robot centric pid
                          double robotCYPID = yPid.calculate(xError, 0);
                          double xCmdField =
                              robotCXPid * drivetrain.getState().Pose.getRotation().getCos()
                                  - robotCYPID * drivetrain.getState().Pose.getRotation().getSin();
                          double yCmdField =
                              robotCXPid * drivetrain.getState().Pose.getRotation().getSin()
                                  + robotCYPID * drivetrain.getState().Pose.getRotation().getCos();
                          this.visionBasedX = MathUtil.applyDeadband(xCmdField, 0.01);
                          this.visionBasedY = MathUtil.applyDeadband(yCmdField, 0.01);
                          velocitySetter.accept(
                              new Pair<Double, Double>(visionBasedX, visionBasedY));
                        }))
                .until(
                    () -> {
                      return (this.visionBasedX == 0.0 && this.visionBasedY == 0)
                          || !LimelightHelpers.getTV(Constants.LIMELIGHT_FRONT_NAME)
                          || joystickInput.get();
                    })
                .finallyDo(
                    () -> {
                      velocitySetter.accept(new Pair<Double, Double>(0.0, 0.0));
                    }));

    addCommands(
        // new DriveToTag(velocitySetter, headingSetter, heading, joystickInput, llName, MaxSpeed,
        // drivetrain, 0.7, 0.0), // TODO: Fix temp constants
        driveToTag, new PrepScoreManual(elevator, arm, () -> elevatorHeight));
  }
}
