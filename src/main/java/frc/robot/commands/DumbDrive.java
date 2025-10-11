package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class DumbDrive extends SequentialCommandGroup {

  public DumbDrive(
      BiConsumer<Double, Double> rrVelocitySetter,
      Consumer<Double> headingSetter,
      double heading,
      Supplier<Boolean> joystickInput,
      String llName,
      double MaxSpeed,
      CommandSwerveDrivetrain drivetrain) {
    Pose3d tagPose3dRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(llName);
    if (tagPose3dRobotSpace.getZ() == 0) {
      System.out.println("no tag seen");
      this.cancel();
    }

    addCommands(
        new InstantCommand(
            () -> {
              double targetHeadingDeg =
                  heading - Math.toDegrees(tagPose3dRobotSpace.getRotation().getY());
              headingSetter.accept(targetHeadingDeg);
            }),
        new InstantCommand(() -> System.out.println("set target heading, waiting 0.5s")),
        new WaitCommand(0.5),
        new InstantCommand(
            () -> {
              rrVelocitySetter.accept(0.0, 1.0);
            }), // 0.2 is 0.2mps speed
        new InstantCommand(() -> System.out.println("set horizontal motion, waiting to align")),
        new WaitCommand(
            Math.abs(
                (tagPose3dRobotSpace.getX() - 0.0)
                    / 1.0)), // 0.0 = horizontal offset meters here, 0.2mps
        // new WaitUntilCommand(() -> Math.abs(LimelightHelpers.getTX(llName)) < 2.0),
        new InstantCommand(
            () -> {
              rrVelocitySetter.accept(1.0, 0.0);
            }),
        new InstantCommand(() -> System.out.println("set vertical motion, waiting to align")),
        new WaitCommand(
            Math.abs((tagPose3dRobotSpace.getZ() - 0.21) / 1.0)), // 0.2mps, 0.21m from tag
        new InstantCommand(
            () -> {
              System.out.println((tagPose3dRobotSpace.getZ() - 0.21) / 1.0);
            }),
        // new WaitUntilCommand(()->{return Math.abs((tagPose3dRobotSpace.getZ() - 0.21)) < 0.05;}),
        new InstantCommand(
            () -> {
              rrVelocitySetter.accept(0.0, 0.0);
            }),
        new InstantCommand(() -> System.out.println("finished, set to zero")));
  }
}
