package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class AutoScoringPrepSequence extends SequentialCommandGroup {
  public AutoScoringPrepSequence(Elevator elevator, Arm arm, Lights lights) {
    addRequirements(elevator, arm);
    addCommands(
        new InstantCommand(() -> lights.setLEDColor(LightCode.SCORE_PREP)),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.SCORE_L2)),
        new WaitUntilCommand(
            elevator::armMovementAllowed), // do it like this so we don't encounter errors with the
        // encoder missing a tick and never triggering this, also
        // allows us to click the button again if there is an issue
        // and not encounter any problems
        new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.L2)),
        new InstantCommand(() -> lights.setLEDColor(LightCode.READY_TO_SCORE)));
  }
}
