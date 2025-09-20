package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class GoHomeSequenceFake extends SequentialCommandGroup {
  public GoHomeSequenceFake(Climb climb, Elevator elevator, Arm arm, Claw claw, Lights lights) {
    addRequirements(climb, elevator, arm, claw);
    addCommands(
        /* do everything that can be done instantly first. we will have to check with design about interferance zones, but doing it in this order should be optimal and not require zone checking */
        new InstantCommand(() -> lights.setLEDColor(LightCode.HOME)),
        new InstantCommand(() -> claw.stopMotors()),
        new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.HOME)),
        new WaitUntilCommand(arm::atDesiredArmPosition),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)),
        new WaitUntilCommand(elevator::atDesiredPosition));
  }
}
