package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToTag extends SequentialCommandGroup{
    public double vx = 5; //arbitrary non-zero starting value bc who knows how wpilib until() works
    public double vy = 5;
    Pose3d tagOffset;
    public DriveToTag(Consumer<Pair<Double, Double>> velocitySetter, Consumer<Double> headingSetter, double heading, Supplier<Boolean> joystickInput, String llName, double MaxSpeed, CommandSwerveDrivetrain drivetrain, double distFromTagOffset, double horizontalOffset){
        ProfiledPIDController xPid = new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(MaxSpeed*0.2, MaxSpeed));
        ProfiledPIDController yPid = new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(MaxSpeed*0.2, MaxSpeed));       
        if (!LimelightHelpers.getTV(llName)) {
            this.cancel();
        }
        tagOffset = LimelightHelpers.getTargetPose3d_RobotSpace(llName);
        addCommands(
            new InstantCommand(()->{
                Rotation3d tagRot = tagOffset.getRotation();
                headingSetter.accept(heading - Math.toDegrees(tagRot.getY()));
            }),
            new RepeatCommand(
                new InstantCommand(()->{
                    if (LimelightHelpers.getTV(Constants.LIMELIGHT_FRONT_NAME)) {
                        tagOffset = LimelightHelpers.getTargetPose3d_RobotSpace(llName);
                    }
                    double xError = tagOffset.getX() + horizontalOffset; // TODO figure out if adding is left or right
                    double zError = tagOffset.getZ() + distFromTagOffset; 
                    double robotCXPid = -xPid.calculate(zError, 0);
                    double robotCYPID = yPid.calculate(xError, 0);
                    double xCmdField = robotCXPid * drivetrain.getState().Pose.getRotation().getCos() - robotCYPID * drivetrain.getState().Pose.getRotation().getSin();
                    double yCmdField = robotCXPid * drivetrain.getState().Pose.getRotation().getSin() + robotCYPID * drivetrain.getState().Pose.getRotation().getCos();
                    vx = MathUtil.applyDeadband(xCmdField, 0.1);
                    vy = MathUtil.applyDeadband(yCmdField, 0.1);
                    velocitySetter.accept(new Pair<Double, Double>(MathUtil.applyDeadband(vx, 0.1), MathUtil.applyDeadband(vx, 0.1)));
                })
            ).until(()->{
                return (this.vx == 0.0 && this.vy == 0) || joystickInput.get();
            }).finallyDo(()->{
                this.vx = 0.0;
                this.vy = 0.0;
                velocitySetter.accept(new Pair<Double, Double>(MathUtil.applyDeadband(vx, 0.1), MathUtil.applyDeadband(vx, 0.1)));
            })
        );
    }
}
