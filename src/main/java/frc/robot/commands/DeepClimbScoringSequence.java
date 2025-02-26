package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class DeepClimbScoringSequence extends SequentialCommandGroup {
  public DeepClimbScoringSequence(Arm arm, Climb climb, Lights lights) {
    addRequirements(climb);
    addCommands(
        new WaitUntilCommand(() -> climb.atClimbPosition(ClimbPosition.CLIMBING_PREP)),
        new InstantCommand(() -> climb.setServoLocked(true)),
        new InstantCommand(() -> lights.setLEDColor(LightCode.CLIMBING)),
        new InstantCommand(() -> climb.setClimbingPID()),
        new InstantCommand(() -> climb.setDesiredClimbPosition(ClimbPosition.CLIMBING)),
        new InstantCommand(() -> lights.setLEDColor(LightCode.PARTY_MODE)));
  }
}
