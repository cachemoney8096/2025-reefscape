package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class ShallowClimbPrep extends SequentialCommandGroup {
  public ShallowClimbPrep(Elevator elevator, Lights lights) {
    addRequirements(elevator);
    addCommands(
      new InstantCommand(() -> lights.setLEDColor(LightCode.CLIMB_PREP_SHALLOW)),
      new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.SHALLOW_PREP)),
      new InstantCommand(() -> lights.setLEDColor(LightCode.READY_TO_CLIMB)));
  }
}
