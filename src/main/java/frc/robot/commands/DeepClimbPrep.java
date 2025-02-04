package frc.robot.commands;
import java.util.Optional;
import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTableInstance;
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

public class DeepClimbPrep extends SequentialCommandGroup {
    public DeepClimbPrep(Climb climb, IntakeLimelight intakeLimelight, Elevator elevator, Arm arm) {
      addRequirements(climb);
      Double[] climbScoringIds = new Double[] {4.0, 5.0, 14.0, 15.0};
        addCommands(
        new ConditionalCommand(
              new SequentialCommandGroup(
                //TODO drive logic here
                //drive should happen at the same time as the next 5 commands
                new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)),
                new ConditionalCommand(
                  new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.DEEP_CLIMB)), 
                  new InstantCommand(),  
                  () -> {return arm.isArmInInterferenceZone();}),
                new ConditionalCommand(
                  new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)), 
                  new InstantCommand(),  
                  () -> {return climb.isClimbInInterferenceZone();})
              ),
              new SequentialCommandGroup(
                new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)),
                new ConditionalCommand(
                  new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.DEEP_CLIMB)), 
                  new InstantCommand(),  
                  () -> {return arm.isArmInInterferenceZone();}),
                new ConditionalCommand(
                  new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)), 
                  new InstantCommand(),  
                  () -> {return climb.isClimbInInterferenceZone();})
              ), // prep for manual if april tag not detected 
          () -> {return Arrays.asList(climbScoringIds).contains(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0));}
        )
      );
      

      
      

    }
  }