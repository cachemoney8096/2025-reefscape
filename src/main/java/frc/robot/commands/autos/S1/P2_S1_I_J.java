package frc.robot.commands.autos.S1;

import java.util.concurrent.locks.Condition;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeLimelight.IntakeLimelight;
import frc.robot.subsystems.ScoringLimelight.ScoringLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;

public class P2_S1_I_J extends SequentialCommandGroup {
    public P2_S1_I_J(
        DriveSubsystem drive,
        Arm arm,
        Claw claw,
        Elevator elevator,
        IntakeLimelight intakelimelight,
        ScoringLimelight scoringlimelight,
        boolean isRed) {
        addRequirements(drive, arm, claw, elevator, 
                        intakelimelight, scoringlimelight);
        addCommands(
            new ConditionalCommand(
                new PathPlannerAuto("R_2P_S1-I-J"), 
                new PathPlannerAuto("B_2P_S1-I-J"), 
                () -> isRed)
        );
    }
}