package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class PrepScoreAndDrive extends SequentialCommandGroup{
    public enum Location {
        LEFT,
        RIGHT
    }

    public PrepScoreAndDrive(Elevator elevator, Arm arm, Supplier<ElevatorHeight> positionS, Supplier<Location> location, BiConsumer<Double, Double> velocitySetter, Consumer<Double> headingSetter,
            Supplier<Boolean> joystickInput, CommandSwerveDrivetrain drivetrain, String llName,
            Supplier<Double> heading){
        final double distanceOffset = 0.0; //TODO placeholder
        addCommands(
            new PrepScoreManual(elevator, arm, positionS),
            new DriveToTag(velocitySetter, headingSetter, joystickInput, drivetrain, llName, ()->distanceOffset, ()->location.get()==Location.LEFT?0.0:0.0, heading).finallyDo(()->velocitySetter.accept(0.0, 0.0))
        );
    }
}
