package frc.robot.commands.autos.S1;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;

public class P4_S1_I_J_K_L extends SequentialCommandGroup {
  public P4_S1_I_J_K_L(CommandSwerveDrivetrain drive, Arm arm, Claw claw, boolean isRed) {

    addRequirements(drive, arm, claw);
    addCommands(
        new ConditionalCommand(
            new PathPlannerAuto("R_4P_S1-I-J-K-L"),
            new PathPlannerAuto("B_4P_S1-I-J-K-L"),
            () -> isRed));
  }
}
