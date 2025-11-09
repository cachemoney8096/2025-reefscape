package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToTag extends SequentialCommandGroup {
    PIDController xController = new PIDController(1, 0.0, 0.0); // input meters output -1 to 1 (percent direction)
    PIDController yController = new PIDController(1, 0.0, 0.0); // input meters output -1 to 1 (percent direction)
    Pose3d tagPoseRobotSpaceInstance;
    public DriveToTag(BiConsumer<Double, Double> velocitySetter, Consumer<Double> headingSetter,
            Supplier<Boolean> joystickInput, CommandSwerveDrivetrain drivetrain, String llName,
            Supplier<Double> distanceOffset, Supplier<Double> horizontalOffset, Supplier<Double> heading) {
        addCommands(
                new InstantCommand(()->{
                    xController.reset();
                    yController.reset();
                }),
                new InstantCommand(
                        () -> {
                            final Pose3d tagOffset = LimelightHelpers.getTargetPose3d_RobotSpace(
                                    llName);
                            if (tagOffset.getZ() == 0) {
                                tagPoseRobotSpaceInstance = new Pose3d();
                                return;
                            }
                            tagPoseRobotSpaceInstance = tagOffset;
                            final Rotation3d tagRot = tagOffset.getRotation();
                            headingSetter.accept(heading.get() - Math.toDegrees(tagRot.getY()));
                        }),
                new WaitUntilCommand(

                        () -> {
                            final Pose3d tagPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(
                                    llName);
                            if (tagPoseRobotSpaceInstance.getZ() == 0) {
                                return true;
                            }
                            if(tagPoseRobotSpace.getZ() != 0){
                                tagPoseRobotSpaceInstance = tagPoseRobotSpace;
                            }
                            final Pose2d tagPoseRobotSpaceWpiConvention = new Pose2d(
                                    tagPoseRobotSpaceInstance.getZ() - distanceOffset.get(),
                                    -tagPoseRobotSpaceInstance.getX() + horizontalOffset.get(),
                                    Rotation2d.fromDegrees(tagPoseRobotSpaceInstance.getRotation().getY()));
                            // get the ll data in wpi convention, also add offsets
                            final Transform2d tagTransformRobotSpaceWpiConvention = new Transform2d(
                                    new Pose2d(), tagPoseRobotSpaceWpiConvention);
                            final Pose2d robotPoseFieldSpace = drivetrain.getState().Pose;
                            final Pose2d targetPoseFieldSpace = robotPoseFieldSpace
                                    .plus(tagTransformRobotSpaceWpiConvention);
                            final Pose2d currentPose = drivetrain.getState().Pose;
                            double xOutput = xController.calculate(
                                    currentPose.getX(), targetPoseFieldSpace.getX());
                            double yOutput = yController.calculate(
                                    currentPose.getY(), targetPoseFieldSpace.getY());
                                
                            double xOutputClamped = MathUtil.clamp(xOutput, -1.5, 1.5);
                            double yOutputClamped = MathUtil.clamp(yOutput, -1.5, 1.5);
                            if(DriverStation.getAlliance().isPresent()){
                                if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
                                        velocitySetter.accept(
                                    xOutputClamped, yOutputClamped);
                                }
                                else{
                                        velocitySetter.accept(
                                    -xOutputClamped, -yOutputClamped);
                                }
                            }
                            else{
                                velocitySetter.accept(
                                    xOutputClamped, yOutputClamped); //default blue if we are cooked
                            }
                            
                            return (Math.abs(xController.getPositionError()) < 0.01
                                    && Math.abs(yController.getPositionError()) < 0.01)
                                    || joystickInput.get();

                        }));
    }
}
