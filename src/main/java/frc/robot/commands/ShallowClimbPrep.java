package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class ShallowClimbPrep extends SequentialCommandGroup {
  public ShallowClimbPrep(Elevator elevator) {
    addRequirements(elevator);
    addCommands(new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.SHALLOW_PREP)));
  }
}
