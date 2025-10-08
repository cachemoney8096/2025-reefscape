package frc.robot.commands;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TestDrive extends SequentialCommandGroup{
    public TestDrive(Consumer<Pair<Double, Double>> velocitySetter, Consumer<Double> headingSetter, double heading, Supplier<Boolean> joystickInput, String llName, double MaxSpeed, CommandSwerveDrivetrain drivetrain){
        Pose3d tagPose3dRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(llName);
        Rotation2d headingRot = Rotation2d.fromDegrees(heading);
        double targetHeading = heading - Math.toDegrees(tagPose3dRobotSpace.getRotation().getY());
        Pose2d tagPoseFieldSpace = new Pose2d(tagPose3dRobotSpace.getZ()*headingRot.getCos() - tagPose3dRobotSpace.getX()*headingRot.getSin(), tagPose3dRobotSpace.getZ()*headingRot.getSin() + tagPose3dRobotSpace.getX()*headingRot.getCos(), Rotation2d.fromDegrees(targetHeading));
        Pose2d botPoseFieldSpaceAtReading = drivetrain.getState().Pose;
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            botPoseFieldSpaceAtReading,
            tagPoseFieldSpace
        );
        PathConstraints constraints = new PathConstraints(0.3, 0.3, 0.3, 0.3);
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, tagPoseFieldSpace.getRotation()));
        addCommands(
            AutoBuilder.followPath(path).until(
                ()->{
                    return joystickInput.get();
                }
            )
        );
    }
}
