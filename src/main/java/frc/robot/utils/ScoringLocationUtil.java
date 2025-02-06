package frc.robot.utils;

public class ScoringLocationUtil {

    // Height to score at
    public enum ScoreHeight {
        L1,
        L2,
        L3,
        L4
    }

    // Side of reef to score on
    public enum ScoreSide {
        A,
        B,
        C,
        D,
        E,
        F
    }

    // Whether to score on either the left or right peg of a given side
    public enum ScorePeg {
        LEFT,
        RIGHT
    }

    private ScoreHeight scoreHeight = ScoreHeight.L4; // default value
    private ScoreSide scoreSide = ScoreSide.A;
    private ScorePeg scorePeg = ScorePeg.LEFT;

    public void setScoreHeight(ScoreHeight chooseHeight) {
        scoreHeight = chooseHeight;
    }

    public ScoreHeight getScoreHeight() {
        return scoreHeight;
    }  

    public void setScoreSide(ScoreSide chooseSide) {
        scoreSide = chooseSide;
    }  
    
    public ScoreSide getScoreSide() {
        return scoreSide;
    }  

    public void setScorePeg(ScorePeg choosePeg) {
        scorePeg = choosePeg;
    }  
    
    public ScorePeg getScorePeg() {
        return scorePeg;
    }  
    
}
