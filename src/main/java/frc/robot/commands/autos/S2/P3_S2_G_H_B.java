package frc.robot.commands.autos.S2;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;

public class P3_S2_G_H_B extends SequentialCommandGroup {
  public P3_S2_G_H_B(
      CommandSwerveDrivetrain drive, Arm arm, Claw claw, Elevator elevator, boolean isRed) {

    addRequirements(drive, arm, claw, elevator);
    addCommands(
        new ConditionalCommand(
            new PathPlannerAuto("R_3P_S2-G-H-B"),
            new PathPlannerAuto("B_3P_S2-G-H-B"),
            () -> isRed));
  }
}
