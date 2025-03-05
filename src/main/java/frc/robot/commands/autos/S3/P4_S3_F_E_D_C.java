package frc.robot.commands.autos.S3;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeLimelight.IntakeLimelight;
import frc.robot.subsystems.ScoringLimelight.ScoringLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;

public class P4_S3_F_E_D_C extends SequentialCommandGroup {
  public P4_S3_F_E_D_C(
      CommandSwerveDrivetrain drive,
      Arm arm,
      Claw claw,
      Elevator elevator,
      IntakeLimelight intakelimelight,
      ScoringLimelight scoringlimelight,
      boolean isRed) {

    addRequirements(drive, arm, claw, elevator, intakelimelight, scoringlimelight);
    addCommands(
        new ConditionalCommand(
            new PathPlannerAuto("R_4P-S3-F-E-D-C"),
            new PathPlannerAuto("B_4P-S3-F-E-D-C"),
            () -> isRed));
  }
}
