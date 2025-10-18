package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.utils.MatchStateUtil;
import frc.robot.utils.PrepStateUtil;

public class PrepScoreSequence extends SequentialCommandGroup {
  public static int tagId = 0;
  public static Transform2d robotToTag;
  public static Pose2d targetPose;

  public PrepScoreSequence(
      Arm arm,
      Elevator elevator,
      Climb climb,
      PrepStateUtil prepStateUtil,
      MatchStateUtil msu,
      Lights lights) {
    addRequirements(arm, elevator);
    final ArmPosition p;
    final ElevatorHeight height;
    final PrepStateUtil.SCORE_LOCATION location = prepStateUtil.getPrepScoreLocation();
    //     InstantCommand setHeight = new InstantCommand(()->{PrepStateUtil.SCORE_HEIGHT pHeight =
    // prepStateUtil.getPrepScoreHeight();

    //     if (pHeight == PrepStateUtil.SCORE_HEIGHT.L2) {
    //       p = ArmPosition.L2;
    //       height = ElevatorHeight.SCORE_L2;
    //     } else if (pHeight == PrepStateUtil.SCORE_HEIGHT.L3) {
    //       p = ArmPosition.L3;
    //       height = ElevatorHeight.SCORE_L3;
    //     } else if (pHeight == PrepStateUtil.SCORE_HEIGHT.L4) {
    //       p = ArmPosition.L4;
    //       height = ElevatorHeight.SCORE_L4;
    //     } else {
    //       p = ArmPosition.L1;
    //       height = ElevatorHeight.SCORE_L1;
    //     }
    //     System.out.println("arm: " + p + " and elevator: " + height);
    // });

    // SequentialCommandGroup setPositions =
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> elevator.setDesiredPosition(height)),
    //         new WaitUntilCommand(elevator::armMovementAllowed),
    //         new InstantCommand(() -> arm.setDesiredPosition(p)));

    // addCommands(
    //     new SequentialCommandGroup(
    //         new InstantCommand(
    //             () -> elevator.setDesiredPosition(prepStateUtil.getElevatorHeight())),
    //         new WaitUntilCommand(elevator::armMovementAllowed),
    //         new InstantCommand(() -> arm.setDesiredPosition(prepStateUtil.getArmPosition()))),
    //     new InstantCommand(() -> lights.setLEDColor(LightCode.SCORE_PREP)),
    //     /* check for a tag first so we can start driving. fall back onto manual driving */
    //     new ConditionalCommand(
    //         new SequentialCommandGroup(
    //             new InstantCommand(
    //                 () -> {
    //                   drive.driveToPose(drive.getState().Pose, targetPose);
    //                 })),
    //         new InstantCommand(),
    //         () -> {
    //           Optional<Transform2d> robotToTagOptional = scoringLimelight.checkForTag();
    //           if (robotToTagOptional.isPresent()) {
    //             robotToTag = robotToTagOptional.get();
    //             int id =
    //                 (int)
    //                     NetworkTableInstance.getDefault()
    //                         .getTable(ScoringLimelightConstants.SCORING_LIMELIGHT_NAME)
    //                         .getEntry("tid")
    //                         .getDouble(0);
    //             /* map of id -> pair<util tag position id, robot heading> */
    //             /* edit from later james - i think i was tweaking and the angles aren't
    // technically needed. however, it does make it more precise. */
    //             TreeMap<Integer, Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>> map =
    //                 new TreeMap<Integer, Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>>();
    //             map.put(
    //                 20,
    //                 new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(
    //                     ReefAngleCalcUtil.ScoreTagPosition.ID_A, 120.0));
    //             map.put(
    //                 8,
    //                 new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(
    //                     ReefAngleCalcUtil.ScoreTagPosition.ID_A, 120.0));
    //             map.put(
    //                 21,
    //                 new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(
    //                     ReefAngleCalcUtil.ScoreTagPosition.ID_B, 180.0));
    //             map.put(
    //                 7,
    //                 new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(
    //                     ReefAngleCalcUtil.ScoreTagPosition.ID_B, 180.0));
    //             map.put(
    //                 22,
    //                 new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(
    //                     ReefAngleCalcUtil.ScoreTagPosition.ID_C, 240.0));
    //             map.put(
    //                 6,
    //                 new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(
    //                     ReefAngleCalcUtil.ScoreTagPosition.ID_C, 240.0));
    //             map.put(
    //                 17,
    //                 new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(
    //                     ReefAngleCalcUtil.ScoreTagPosition.ID_D, 300.0));
    //             map.put(
    //                 11,
    //                 new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(
    //                     ReefAngleCalcUtil.ScoreTagPosition.ID_D, 300.0));
    //             map.put(
    //                 18,
    //                 new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(
    //                     ReefAngleCalcUtil.ScoreTagPosition.ID_E, 0.0));
    //             map.put(
    //                 10,
    //                 new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(
    //                     ReefAngleCalcUtil.ScoreTagPosition.ID_E, 0.0));
    //             map.put(
    //                 19,
    //                 new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(
    //                     ReefAngleCalcUtil.ScoreTagPosition.ID_F, 60.0));
    //             map.put(
    //                 9,
    //                 new Pair<ReefAngleCalcUtil.ScoreTagPosition, Double>(
    //                     ReefAngleCalcUtil.ScoreTagPosition.ID_F, 60.0));
    //             if (map.containsKey(id)) {
    //               Translation2d offset =
    //                   ReefAngleCalcUtil.translateScorePositionOffset(
    //                       map.get(id).getFirst(), location ==
    // PrepStateUtil.SCORE_LOCATION.RIGHT);
    //               robotToTag = robotToTag.plus(new Transform2d(offset, new Rotation2d()));
    //               targetPose = drive.getState().Pose.plus(robotToTag);
    //               targetPose =
    //                   new Pose2d(
    //                       targetPose.getTranslation(), new Rotation2d(map.get(id).getSecond()));
    //             } else {
    //               return false;
    //             }
    //             return false;
    //           }
    //           return false;
    //         }),
    //     new InstantCommand(() -> lights.setLEDColor(LightCode.READY_TO_SCORE)));
  }
}
