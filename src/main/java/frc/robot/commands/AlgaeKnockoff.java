package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;

public class AlgaeKnockoff extends SequentialCommandGroup {
  public AlgaeKnockoff(Elevator elevator, DriveSubsystem drive) {
    addRequirements(elevator, drive);
    addCommands(
        new InstantCommand(() -> elevator.setDesiredPosition(Elevator.ElevatorHeight.ALGAE)));
    
  }
}
