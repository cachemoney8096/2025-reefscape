package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class ShallowClimb extends SequentialCommandGroup {
  public ShallowClimb(Elevator elevator, Lights lights) {
    addRequirements(elevator);
    addCommands(
        new WaitUntilCommand(() -> elevator.atElevatorPosition(ElevatorHeight.SHALLOW_PREP)),
        new InstantCommand(() -> lights.setLEDColor(LightCode.CLIMBING)),
        new InstantCommand(() -> elevator.setControlParams(false)),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.SHALLOW_CLIMB)),
        new InstantCommand(() -> lights.setLEDColor(LightCode.PARTY_MODE)));
  }
}
