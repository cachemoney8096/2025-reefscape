package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;

public class AlgaeKnockoff extends SequentialCommandGroup {
  public AlgaeKnockoff(Elevator elevator) {
    addRequirements(elevator);
    addCommands(
        new InstantCommand(() -> elevator.setDesiredPosition(Elevator.ElevatorHeight.ALGAE)));
  }
}
