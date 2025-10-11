package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PIDToPoint extends SequentialCommandGroup {
  PIDController xController =
      new PIDController(1, 0.0, 0.0); // input meters output -1 to 1 (percent direction)
  PIDController yController =
      new PIDController(1, 0.0, 0.0); // input meters output -1 to 1 (percent direction)
  final double feedforwardOutput = 0.2; // feed forward output
  Pose3d tagPoseRobotSpace;
  Pose2d robotPoseFieldSpace;
  Pose2d targetPoseFieldSpace;

  static double clamp(double value, double threshold) {
    if (value > threshold) {
      return threshold;
    }
    if (value < -threshold) {
      return -threshold;
    }
    return value;
  }

  public PIDToPoint(
      Consumer<Pair<Double, Double>> velocitySetter,
      Consumer<Double> headingSetter,
      double heading,
      Supplier<Boolean> joystickInput,
      String llName,
      CommandSwerveDrivetrain drivetrain,
      double distanceOffsetMeters,
      double horizontalOffsetMeters) {
    // don't require the drive, we control it in robot container
    addCommands(
        // grab information
        new InstantCommand(
            () -> {
              tagPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(llName);
              if (tagPoseRobotSpace.getZ() == 0) {
                System.out.println("no tag detected");
                return; // stop
              }
              final Pose2d tagPoseRobotSpaceWpiConvention =
                  new Pose2d(
                      tagPoseRobotSpace.getZ() - distanceOffsetMeters,
                      tagPoseRobotSpace.getX() + horizontalOffsetMeters,
                      Rotation2d.fromDegrees(tagPoseRobotSpace.getRotation().getY()));
              // get the ll data in wpi convention, also add offsets
              final Transform2d tagTransformRobotSpaceWpiConvention =
                  new Transform2d(new Pose2d(), tagPoseRobotSpaceWpiConvention);
              robotPoseFieldSpace = drivetrain.getState().Pose;
              targetPoseFieldSpace = robotPoseFieldSpace.plus(tagTransformRobotSpaceWpiConvention);
              System.out.println(
                  "vector: x=" + targetPoseFieldSpace.getX() + " y=" + targetPoseFieldSpace.getY());
            }),
        new InstantCommand(
            () -> {
              /*final Pose3d tagOffset = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME);
              final Rotation3d tagRot = tagOffset.getRotation();
              System.out.println("heading tag robot space" + Math.toDegrees(tagRot.getY()));
              headingSetter.accept(heading - Math.toDegrees(tagRot.getY()));
              System.out.println("heading" + (heading - Math.toDegrees(tagRot.getY())));*/
              headingSetter.accept(heading + LimelightHelpers.getTX(llName));
            }),
        new WaitUntilCommand(
            () -> {
              if (tagPoseRobotSpace.getZ() == 0) {
                System.out.println("no tag detected");
                return true; // end immediately
              }
              final Pose2d currentPose = drivetrain.getState().Pose;
              double xOutput =
                  xController.calculate(currentPose.getX(), targetPoseFieldSpace.getX());
              double yOutput =
                  yController.calculate(currentPose.getY(), targetPoseFieldSpace.getY());
              final double xPosErrorMeters = xController.getError();
              final double yPosErrorMeters = yController.getError();
              final Translation2d errorMeters = new Translation2d(xPosErrorMeters, yPosErrorMeters);
              final Translation2d errorDirection = errorMeters.div(errorMeters.getNorm());
              final double xFfOutput = errorDirection.getX() * feedforwardOutput;
              final double yFfOutput = errorDirection.getY() * feedforwardOutput;
              xOutput += xFfOutput;
              yOutput += yFfOutput;
              double xOutputClamped = clamp(xOutput, 1.0);
              double yOutputClamped = clamp(yOutput, 1.0);
              velocitySetter.accept(new Pair<Double, Double>(xOutputClamped, yOutputClamped));
              return (Math.abs(xController.getError()) < 0.01
                      && Math.abs(yController.getError()) < 0.01)
                  || joystickInput.get();
            }));
  }
}
