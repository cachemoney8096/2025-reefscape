package frc.robot.utils;

import java.util.TreeMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

// assuming heading is correct (facing head on)
public class ReefAngleCalcUtil {

    public enum ScoreTagPosition {
        ID_A, // ID 20 || ID 8
        ID_B, // ID 21 || ID 7
        ID_C, // ID 22 || ID 6
        ID_D, // ID 17 || ID 11
        ID_F, // ID 18 || ID 10
        ID_G, // ID 19 || ID 9
    }

    private static final double scoreTagOffsetInches = Constants.REEF_SCORING_OFFSET_INCHES;

    // Where Pair<Double, Double> -> Vector<X, Y>
    private static TreeMap<ScoreTagPosition, Pair<Double, Double>> tagScoreVectorMap;
    private static boolean isRight;
    private static ScoreTagPosition desiredTagScorePosition;

    /**
     * (Reef is symmetric on both sides of the field).
     * Assume ID_A starts at at ID20 on Blue and ID8 on Red, and the tags go
     * clockwise around the reef.
     * 
     * @param desiredTagScorePosition ScoreTagPosition enum
     * @param isRight                 Determine if angle is negative or positive
     * 
     */
    private static void init(ScoreTagPosition desiredTagScorePosition, boolean isRight) {
        ReefAngleCalcUtil.isRight = isRight;
        ReefAngleCalcUtil.desiredTagScorePosition = desiredTagScorePosition;
        tagScoreVectorMap = new TreeMap<ScoreTagPosition, Pair<Double, Double>>();
        tagScoreVectorMap.put(ScoreTagPosition.ID_A,
                new Pair<Double, Double>((scoreTagOffsetInches / 2) * Math.sqrt(3), (scoreTagOffsetInches / 2)));
        tagScoreVectorMap.put(ScoreTagPosition.ID_B,
                new Pair<Double, Double>((0.0), (scoreTagOffsetInches)));
        tagScoreVectorMap.put(ScoreTagPosition.ID_C,
                new Pair<Double, Double>((scoreTagOffsetInches / 2) * Math.sqrt(3), (scoreTagOffsetInches / 2)));
        tagScoreVectorMap.put(ScoreTagPosition.ID_D,
                new Pair<Double, Double>((scoreTagOffsetInches / 2) * Math.sqrt(3), (scoreTagOffsetInches / 2)));
        tagScoreVectorMap.put(ScoreTagPosition.ID_F,
                new Pair<Double, Double>((0.0), (scoreTagOffsetInches)));
        tagScoreVectorMap.put(ScoreTagPosition.ID_G,
                new Pair<Double, Double>((scoreTagOffsetInches / 2) * Math.sqrt(3), (scoreTagOffsetInches / 2)));
    }

    public static Translation2d translateScorePositionOffset(ScoreTagPosition desiredTagScorePosition, boolean isRight) {
        init(desiredTagScorePosition, isRight);

        Pair<Double, Double> calcVector = tagScoreVectorMap.get(ReefAngleCalcUtil.desiredTagScorePosition);
        Pair<Double, Double> updatedVector = new Pair<Double, Double>(null, null);
        switch (desiredTagScorePosition) {
            case ID_A:
                updatedVector = isRight ? new Pair<Double, Double>(-calcVector.getFirst(), calcVector.getSecond())
                        : new Pair<Double, Double>(calcVector.getFirst(), -calcVector.getSecond());
            case ID_B:
                updatedVector = isRight ? new Pair<Double, Double>(calcVector.getFirst(), calcVector.getSecond())
                        : new Pair<Double, Double>(calcVector.getFirst(), -calcVector.getSecond());
            case ID_C:
                updatedVector = isRight ? new Pair<Double, Double>(calcVector.getFirst(), calcVector.getSecond())
                        : new Pair<Double, Double>(-calcVector.getFirst(), -calcVector.getSecond());
            case ID_D:
                updatedVector = isRight ? new Pair<Double, Double>(calcVector.getFirst(), -calcVector.getSecond())
                        : new Pair<Double, Double>(-calcVector.getFirst(), calcVector.getSecond());
            case ID_F:
                updatedVector = isRight ? new Pair<Double, Double>(calcVector.getFirst(), -calcVector.getSecond())
                        : new Pair<Double, Double>(calcVector.getFirst(), calcVector.getSecond());
            case ID_G:
                updatedVector = isRight ? new Pair<Double, Double>(-calcVector.getFirst(), -calcVector.getSecond())
                        : new Pair<Double, Double>(calcVector.getFirst(), calcVector.getSecond());
            default:
                break;
        }

        return new Translation2d(updatedVector.getFirst(), updatedVector.getSecond());
    }
}
