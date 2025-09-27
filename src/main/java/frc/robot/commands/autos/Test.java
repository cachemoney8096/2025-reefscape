package frc.robot.commands.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;

public class Test extends SequentialCommandGroup {
  public Test(
      CommandSwerveDrivetrain drive, boolean isRed) {

    addRequirements(drive);
    addCommands(
        new ConditionalCommand(
            new PathPlannerAuto("PUSH AUTO"), new PathPlannerAuto("PUSH AUTO"), () -> isRed));
  }
}
