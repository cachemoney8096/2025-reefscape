package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;

public class DeepClimbPrep extends SequentialCommandGroup {
  public DeepClimbPrep(Climb climb) {
    addRequirements(climb);
    // TODO use apriltag to allign with cage
    addCommands(
        new WaitUntilCommand(() -> climb.isClimbMovable()),
        new InstantCommand(() -> climb.setDesiredClimbPosition(ClimbPosition.CLIMBING_PREP)));
  }
}
