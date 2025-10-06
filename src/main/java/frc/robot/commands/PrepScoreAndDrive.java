package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class PrepScoreAndDrive extends ParallelCommandGroup {
    public enum Location {
        LEFT,
        RIGHT
    }

    public PrepScoreAndDrive(Elevator elevator, Arm arm, ElevatorHeight elevatorHeight, Consumer<Pair<Double, Double>> velocitySetter, Consumer<Double> headingSetter, double heading, Supplier<Boolean> joystickInput, String llName, double MaxSpeed, CommandSwerveDrivetrain drivetrain, Location location) {
        addCommands(
            new DriveToTag(velocitySetter, headingSetter, heading, joystickInput, llName, MaxSpeed, drivetrain, 0.7, 0.0), // TODO: Fix temp constants
            new PrepScoreManual(elevator, arm, elevatorHeight)
        );
    }
}
