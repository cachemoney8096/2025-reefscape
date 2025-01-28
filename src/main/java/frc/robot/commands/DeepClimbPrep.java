package frc.robot.commands;
import java.util.Optional;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.IntakeLimelight.IntakeLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import edu.wpi.first.math.geometry.Transform2d;

public class DeepClimbPrep extends SequentialCommandGroup {
    private Optional<Transform2d> isTag = Optional.empty();
    public DeepClimbPrep(Climb climb, IntakeLimelight intakeLime, Elevator elevator, Arm arm) {
      addRequirements(climb);
      isTag = intakeLime.checkForTag();

      addCommands(
        new ConditionalCommand(
              new SequentialCommandGroup(
                //TODO drive logic here
                //drive should happen at the same time as the next 5 commands
                new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)),
                new WaitUntilCommand(() -> arm.isArmMoveable()),
                new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.DEEP_CLIMB)),
                new WaitUntilCommand(() -> climb.isClimbMovable()),
                new InstantCommand(() -> climb.setDesiredClimbPosition(ClimbPosition.CLIMBING_PREP))
              ),
          new InstantCommand(), // return out of the command if the robot does not see the intake april tag 
          () -> {return !isTag.isEmpty();}
        )
      );

    }
  }