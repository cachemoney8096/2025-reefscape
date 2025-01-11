package frc.robot.subsystems.elevator;
import java.util.TreeMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    public enum ElevatorHeight{
        HOME,
        SCORE_L4,
        SCORE_L3,
        SCORE_L2,
        SCORE_L1,
        PRE_DEEPCLIMB,
        SCORE_DEEPCLIMB,
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

    public void setDesiredHeight(ElevatorHeight desiredPos) {
        leftElevatorMotor.setPosition(elevatorPositions.get(desiredPos));
    }
}