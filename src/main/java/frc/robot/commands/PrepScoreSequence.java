package frc.robot.commands;

import java.util.Arrays;
import java.util.stream.IntStream;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ScoringLimelight.ScoringLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class PrepScoreSequence extends SequentialCommandGroup {
    public PrepScoreSequence(Arm arm, Elevator elevator, ScoringLimelight scoringLimelight, ArmPosition armPosition,
            boolean isRight) {
        addRequirements(arm, elevator, scoringLimelight);
        Double[] reefScoringIds = new Double[] {17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0};
        // boolean check = 
        addCommands(
                /**
                 * Set Desired Arm Pos
                 * Set Desired Elevator Pos
                 * 
                 */
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> elevator.setDesiredPosition(ElevatorHeight.values()[Constants.PLACEHOLDER_INT])),
                        new InstantCommand(
                                () -> arm.setDesiredPosition(ArmPosition.values()[Constants.PLACEHOLDER_INT]))),
                /**
                 * If tag is seen, use limelight command sequence
                 * Otherwise, assume manual
                 */
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        
                    ), 
                    new SequentialCommandGroup(

                    ), 
                    
                    () -> (scoringLimelight.checkForTag().isPresent() && Arrays.asList(reefScoringIds).contains(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0))))
                    );
    }
}
