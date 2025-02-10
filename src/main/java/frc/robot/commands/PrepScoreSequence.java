package frc.robot.commands;

import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.utils.MatchStateUtil;
import frc.robot.utils.ReefAngleCalcUtil;

public class PrepScoreSequence extends SequentialCommandGroup {
    public static int tagId = 0;
    public static Transform2d robotToTag;
    public static Pose2d targetPose;
  public PrepScoreSequence(
      Arm arm,
      Elevator elevator,
      ScoringLimelight scoringLimelight,
      Climb climb,
      ElevatorHeight height, 
      RobotContainer.ScoringLocation location,
      DriveSubsystem drive,
      MatchStateUtil msu) {
    addRequirements(arm, elevator, scoringLimelight);

    final ArmPosition p;
    if(height == ElevatorHeight.SCORE_L2){
        p = ArmPosition.L2;
    }
    else if(height == ElevatorHeight.SCORE_L3){
        p = ArmPosition.L3;
    }
    else if(height == ElevatorHeight.SCORE_L4){
        p = ArmPosition.L4;
    }
    else{
        p = ArmPosition.L1;
    }

    SequentialCommandGroup setPositions = new SequentialCommandGroup(
        new InstantCommand(()->arm.setDesiredPosition(p)),
        new InstantCommand(()->elevator.setDesiredPosition(height))
    );
    
    SequentialCommandGroup checkAndSetPositions = new SequentialCommandGroup(
        new ConditionalCommand(new SequentialCommandGroup(
            new InstantCommand(()->climb.setDesiredClimbPosition(ClimbPosition.STOWED)),
            new WaitUntilCommand(()->climb.atDesiredPosition())), new InstantCommand(), ()->!arm.isArmInInterferenceZone()),
       setPositions
    );

    addCommands(
        /* check for a tag first so we can start driving. fall back onto manual driving */
        new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(()->drive.driveToPoint(targetPose))
            ),
            new InstantCommand(),
            ()->{
                Optional<Transform2d> robotToTagOptional = scoringLimelight.checkForTag();
                if(robotToTagOptional.isPresent()){
                    robotToTag = robotToTagOptional.get();
                    int id = (int)NetworkTableInstance.getDefault()
                            .getTable("limelight-scoring")
                            .getEntry("tid")
                            .getDouble(0);
                    /* map of id -> pair<util tag position id, robot heading> */
                    /* edit from later james - i think i was tweaking and the angles aren't technically needed. however, it does make it more precise. */
                    TreeMap<Integer, Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>> map = new TreeMap<Integer, Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>>();
                    map.put(20, new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(ReefAngleCalcUtil.ScoreTagPosition.ID_A, 120.0));
                    map.put(8, new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(ReefAngleCalcUtil.ScoreTagPosition.ID_A, 120.0));
                    map.put(21, new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(ReefAngleCalcUtil.ScoreTagPosition.ID_B, 180.0));
                    map.put(7, new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(ReefAngleCalcUtil.ScoreTagPosition.ID_B, 180.0));
                    map.put(22, new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(ReefAngleCalcUtil.ScoreTagPosition.ID_C, 240.0));
                    map.put(6, new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(ReefAngleCalcUtil.ScoreTagPosition.ID_C, 240.0));
                    map.put(17, new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(ReefAngleCalcUtil.ScoreTagPosition.ID_D, 300.0));
                    map.put(11, new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(ReefAngleCalcUtil.ScoreTagPosition.ID_D, 300.0));
                    map.put(18, new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(ReefAngleCalcUtil.ScoreTagPosition.ID_E, 0.0));
                    map.put(10, new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(ReefAngleCalcUtil.ScoreTagPosition.ID_E, 0.0));
                    map.put(19, new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(ReefAngleCalcUtil.ScoreTagPosition.ID_F, 60.0));
                    map.put(9, new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(ReefAngleCalcUtil.ScoreTagPosition.ID_F, 60.0));
                    if(map.containsKey(id)){
                        Translation2d offset = ReefAngleCalcUtil.translateScorePositionOffset(map.get(id).getFirst(), location==RobotContainer.ScoringLocation.RIGHT);
                        robotToTag = robotToTag.plus(new Transform2d(offset, new Rotation2d()));
                        targetPose = drive.getPose().plus(robotToTag);
                        targetPose = new Pose2d(targetPose.getTranslation(), new Rotation2d(map.get(id).getSecond()));
                    }
                    else{
                        return false;
                    }
                    return true;
                }
                return false;
            }
        ),
        checkAndSetPositions
    );
  }
}
