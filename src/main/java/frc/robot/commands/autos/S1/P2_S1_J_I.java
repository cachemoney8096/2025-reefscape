package frc.robot.commands.autos.S1;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;

public class P2_S1_J_I extends SequentialCommandGroup {
  public P2_S1_J_I(
      CommandSwerveDrivetrain drive, Arm arm, Claw claw, Elevator elevator, boolean isRed) {

    addRequirements(drive, arm, claw, elevator);
    addCommands(
        new ConditionalCommand(
            new PathPlannerAuto("R_2P_S1-J-I"), new PathPlannerAuto("B_2P_S1-J-I"), () -> isRed));
  }
}
