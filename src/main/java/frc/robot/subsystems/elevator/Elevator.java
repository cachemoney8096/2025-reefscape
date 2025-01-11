package frc.robot.subsystems.elevator;
import java.util.Optional;
import java.util.TreeMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
    public enum ElevatorHeight{
        HOME,
        SCORE_L4,
        SCORE_L3,
        SCORE_L2,
        SCORE_L1,
        PRE_SHALLOWCLIMB,
        SCORE_SHALLOWCLIMB;
    }

    
    public TalonFX leftElevatorMotor = new TalonFX(ElevatorConstants.LEFT_ELEVATOR_MOTOR);
    public TalonFX rightElevatorMotor = new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR);
    private TreeMap<ElevatorHeight, Double> elevatorPositions = new TreeMap<ElevatorHeight, Double>();;

    public Elevator() {
        
        elevatorPositions.put(ElevatorHeight.HOME, ElevatorCal.POSITION_HOME_INCHES);
        elevatorPositions.put(ElevatorHeight.SCORE_L4, ElevatorCal.POSITION_SCORE_L4_INCHES);
        elevatorPositions.put(ElevatorHeight.SCORE_L3, ElevatorCal.POSITION_SCORE_L3_INCHES);
        elevatorPositions.put(ElevatorHeight.SCORE_L2, ElevatorCal.POSITION_SCORE_L2_INCHES);
        elevatorPositions.put(ElevatorHeight.SCORE_L1, ElevatorCal.POSITION_SCORE_L1_INCHES);
        elevatorPositions.put(ElevatorHeight.PRE_SHALLOWCLIMB, ElevatorCal.POSITION_PRE_SHALLOWCLIMB_INCHES);
        elevatorPositions.put(ElevatorHeight.SCORE_SHALLOWCLIMB, ElevatorCal.POSITION_SCORE_SHALLOWCLIMB_INCHES);
        initTalons();
    }

     private void initTalons() {
        TalonFXConfigurator cfgLeft = leftElevatorMotor.getConfigurator();
        TalonFXConfigurator cfgRight = rightElevatorMotor.getConfigurator();

        TalonFXConfiguration toApply = new TalonFXConfiguration();
        toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO change this to make run intake methods work
        toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast; //TODO change this based on how tight the compression is
        toApply.CurrentLimits.SupplyCurrentLimit = ElevatorCal.ELEVATOR_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS;
        toApply.CurrentLimits.StatorCurrentLimit = ElevatorCal.ELEVATOR_MOTOR_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        toApply.CurrentLimits.StatorCurrentLimitEnable = true;
        toApply.Slot0.kP = ElevatorCal.ELEVATOR_MOTOR_P;
        toApply.Slot0.kI = ElevatorCal.ELEVATOR_MOTOR_I;
        toApply.Slot0.kD = ElevatorCal.ELEVATOR_MOTOR_D;
        toApply.Slot0.kV = ElevatorCal.ELEVATOR_MOTOR_FF;
        cfgLeft.apply(toApply);
        cfgRight.apply(toApply);
        Follower master = new Follower(leftElevatorMotor.getDeviceID(), true);
        rightElevatorMotor.setControl(master);
    }

    private ProfiledPIDController ElevatorController =
      new ProfiledPIDController(
          ElevatorCal.ELEVATOR_P,
          ElevatorCal.ELEVATOR_I,
          ElevatorCal.ELEVATOR_D,
          new TrapezoidProfile.Constraints(
              ElevatorCal.MAX_VELOCITY_IN_PER_SECOND,
              ElevatorCal.MAX_ACCELERATION_IN_PER_SECOND_SQUARED));

    private ProfiledPIDController currentPIDController = ElevatorController;
    private double elevatorDemandVoltsA = 0.0;
    private double elevatorDemandVoltsB = 0.0;
    private double elevatorDemandVoltsC = 0.0;
    private double desiredSetpointPosition = 0.0;
    private double desiredSetpointVelocity = 0.0;

    private Optional<Double> prevTimestamp = Optional.empty();
    private SimpleMotorFeedforward currentFeedforward = ElevatorCal.ELEVATOR_FF;
    private double prevVelocityInPerSec = 0;
    private ElevatorHeight desiredPosition = ElevatorHeight.HOME;
    public TalonFX leftMotor = new TalonFX(RobotMap.LEFT_ELEVATOR_CAN_ID);
    public TalonFX rightMotor = new TalonFX(RobotMap.RIGHT_ELEVATOR_CAN_ID);

    private boolean nearHome() {
        return leftMotor.getPosition().getValueAsDouble() * ElevatorConstants.GEAR_RATIO_CIRCUM < (elevatorPositions.get(ElevatorHeight.HOME) + 1.0);
      }
      

    private void controlPosition(double inputPositionInch) {
        currentPIDController.setGoal(inputPositionInch); // set the pid controller's goal
        elevatorDemandVoltsA =
            currentPIDController.calculate(
                leftMotor
                    .getPosition().getValueAsDouble() * ElevatorConstants.GEAR_RATIO_CIRCUM); // calculate the pid output based on the goal and the current
        // position
        final double timestamp = Timer.getFPGATimestamp(); // get the current timestamp
        final double nextVelocityInPerSec =
            currentPIDController.getSetpoint().velocity; // calculate the next velocity we want to be at
        if (prevTimestamp.isPresent()) {
            currentFeedforward.calculate(nextVelocityInPerSec)
            elevatorDemandVoltsB = currentFeedforward.calculateWithVelocities(prevVelocityInPerSec, nextVelocityInPerSec);
      // based on how velocity has changed, calculate a feedforward factor.
        // for example, if it didn't change as much as we expect, we could be
        // working against gravity.
        } else {
        elevatorDemandVoltsB =
            currentFeedforward.calculate(
                nextVelocityInPerSec); // if we don't have a past velocity, we can't calculate a
        // feedforward factor using acceleration
        }
        elevatorDemandVoltsC =
           ElevatorCal.ELEVATOR_KS; // make sure we are applying enough voltage to move at all

        desiredSetpointPosition =
            currentPIDController.getSetpoint().position; // these are for debug on shuffleboard
        desiredSetpointVelocity = currentPIDController.getSetpoint().velocity;

        double voltageToSet =
            elevatorDemandVoltsA + elevatorDemandVoltsB + elevatorDemandVoltsC; // add it all up
        if (desiredPosition == ElevatorHeight.HOME && nearHome()) {
        voltageToSet =
            ElevatorCal
                .ELEVATOR_KS; // if we are pretty close to home, we aren't going to execute a pid
        // curve, and are just going to "taxi in" rather than "taking off and
        // landing"
        }

        leftMotor.setVoltage(voltageToSet); // apply the voltage
        rightMotor.setVoltage(voltageToSet);

        prevVelocityInPerSec =
            nextVelocityInPerSec; // set our now previous velocity and timestamp for future calculations
        prevTimestamp = Optional.of(timestamp);
   }
   
}