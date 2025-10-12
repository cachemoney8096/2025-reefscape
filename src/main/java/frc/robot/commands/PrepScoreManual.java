package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import java.util.function.Supplier;

public class PrepScoreManual extends SequentialCommandGroup {

  public PrepScoreManual(Elevator elevator, Arm arm, Supplier<ElevatorHeight> positionS) {

    addCommands(
        new InstantCommand(
            () -> {
              elevator.setDesiredPosition(ElevatorHeight.ARM_CLEAR_OF_CLIMB);
            }),
        new WaitUntilCommand(elevator::atDesiredPosition),
        new InstantCommand(
            () -> {
              ElevatorHeight position = positionS.get();
              ArmPosition armPosition;
              switch (position) {
                case SCORE_L1:
                  armPosition = ArmPosition.L1;
                  break;
                case SCORE_L2:
                  armPosition = ArmPosition.L2;
                  break;
                case SCORE_L3:
                  armPosition = ArmPosition.L3;
                  break;
                default:
                  armPosition = ArmPosition.L1;
                  break;
              }

              elevator.setDesiredPosition(position);
              arm.setDesiredPosition(armPosition);
            }));

    addRequirements(elevator, arm);
  }
}
