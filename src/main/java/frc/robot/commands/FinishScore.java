package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;

public class FinishScore extends SequentialCommandGroup {
  public FinishScore(Claw claw, Elevator elevator, Arm arm, ElevatorHeight height, Lights lights) {
    addCommands(
        new InstantCommand(() -> System.out.println("entered finishscore")),
        new InstantCommand(() -> System.out.println("passed wait")),
        new InstantCommand(() -> claw.runMotorsScoring()),
        new WaitCommand(1.0),
        new InstantCommand(() -> claw.stopMotors()),
        new InstantCommand(() -> lights.setLEDColor(LightCode.OFF)));
  }
}
