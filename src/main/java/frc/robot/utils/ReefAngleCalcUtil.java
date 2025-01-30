package frc.robot.utils;

import java.util.TreeMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

// assuming heading is correct (facing head on)
public class ReefAngleCalcUtil {
    private double distanceFromTag = 0;

    public enum ScoreTagPosition {
        ID1, // ID 20 || ID 8
        ID2, // ID 21 || ID 7g
        ID3, // ID 22 || ID 6
        ID4, // ID 17 || ID 11
        ID5, // ID 18 || ID 10
        ID6 // ID 19 || ID 9
    }

    private final double scoreTagOffsetInches = Constants.REEF_SCORING_OFFSET_INCHES;

    // Where Pair<Double, Double> -> Vector<X, Y>
    private TreeMap<ScoreTagPosition, Pair<Double, Double>> tagScoreVectorMap;
    private boolean isRight;
    private ScoreTagPosition desiredTagScorePosition;

    /**
     * (Reef is symmetric on both sides of the field).
     * Assume ID1 starts at at ID20 on Blue and ID8 on Red, and the tags go
     * clockwise around the reef.
     * 
     * @param desiredTagScorePosition ScoreTagPosition enum
     * @param isRight                 Determine if angle is negative or positive
     * 
     */
    public ReefAngleCalcUtil(ScoreTagPosition desiredTagScorePosition, boolean isRight) {
        this.isRight = isRight;
        this.desiredTagScorePosition = desiredTagScorePosition;
        tagScoreVectorMap = new TreeMap<ScoreTagPosition, Pair<Double, Double>>();
        tagScoreVectorMap.put(ScoreTagPosition.ID1,
                new Pair<Double, Double>((scoreTagOffsetInches / 2) * Math.sqrt(3), (scoreTagOffsetInches / 2)));
        tagScoreVectorMap.put(ScoreTagPosition.ID2,
                new Pair<Double, Double>((0.0), (scoreTagOffsetInches)));
        tagScoreVectorMap.put(ScoreTagPosition.ID3,
                new Pair<Double, Double>((scoreTagOffsetInches / 2) * Math.sqrt(3), (scoreTagOffsetInches / 2)));
        tagScoreVectorMap.put(ScoreTagPosition.ID4,
                new Pair<Double, Double>((scoreTagOffsetInches / 2) * Math.sqrt(3), (scoreTagOffsetInches / 2)));
        tagScoreVectorMap.put(ScoreTagPosition.ID5,
                new Pair<Double, Double>((0.0), (scoreTagOffsetInches)));
        tagScoreVectorMap.put(ScoreTagPosition.ID6,
                new Pair<Double, Double>((scoreTagOffsetInches / 2) * Math.sqrt(3), (scoreTagOffsetInches / 2)));
    }

    public Translation2d translateScorePositionOffset() {
        Pair<Double, Double> calcVector = tagScoreVectorMap.get(this.desiredTagScorePosition);
        Pair<Double, Double> updatedVector = new Pair<Double, Double>(null, null);
        switch (desiredTagScorePosition) {
            case ID1:
                updatedVector = isRight ? new Pair<Double, Double>(-calcVector.getFirst(), calcVector.getSecond())
                        : new Pair<Double, Double>(calcVector.getFirst(), -calcVector.getSecond());
            case ID2:
                updatedVector = isRight ? new Pair<Double, Double>(calcVector.getFirst(), calcVector.getSecond())
                        : new Pair<Double, Double>(calcVector.getFirst(), -calcVector.getSecond());
            case ID3:
                updatedVector = isRight ? new Pair<Double, Double>(calcVector.getFirst(), calcVector.getSecond())
                        : new Pair<Double, Double>(-calcVector.getFirst(), -calcVector.getSecond());
            case ID4:
                updatedVector = isRight ? new Pair<Double, Double>(calcVector.getFirst(), -calcVector.getSecond())
                        : new Pair<Double, Double>(-calcVector.getFirst(), calcVector.getSecond());
            case ID5:
                updatedVector = isRight ? new Pair<Double, Double>(calcVector.getFirst(), -calcVector.getSecond())
                        : new Pair<Double, Double>(calcVector.getFirst(), calcVector.getSecond());
            case ID6:
                updatedVector = isRight ? new Pair<Double, Double>(-calcVector.getFirst(), -calcVector.getSecond())
                        : new Pair<Double, Double>(calcVector.getFirst(), calcVector.getSecond());
            default:
                break;
        }

        return new Translation2d(updatedVector.getFirst(), updatedVector.getSecond());
    }
}
