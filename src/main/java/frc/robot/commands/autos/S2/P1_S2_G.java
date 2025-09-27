package frc.robot.commands.autos.S2;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;

public class P1_S2_G extends SequentialCommandGroup {
  public P1_S2_G(
      CommandSwerveDrivetrain drive,
      Arm arm,
      Claw claw,
      Elevator elevator,
      boolean isRed) {

    addRequirements(drive, arm, claw, elevator);
    addCommands(
        new ConditionalCommand(
            new PathPlannerAuto("R_1P_S2-G"), new PathPlannerAuto("B_1P_S2-G"), () -> isRed));
  }
}
