package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class ShallowUnclimb extends SequentialCommandGroup {
  public ShallowUnclimb(Elevator elevator, Lights lights) {
    addRequirements(elevator);
    addCommands(
        new InstantCommand(() -> elevator.setControlParams(false)),
        new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.SHALLOW_PREP)),
        new InstantCommand(() -> elevator.setControlParams(true)),
        new InstantCommand(() -> lights.setLEDColor(LightCode.OFF)));
  }
}
