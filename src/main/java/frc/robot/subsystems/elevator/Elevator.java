package frc.robot.subsystems.elevator;

import java.util.Optional;
import java.util.TreeMap;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.SendableHelper;
import edu.wpi.first.util.sendable.SendableBuilder;


public class Elevator extends SubsystemBase {
    public enum ElevatorHeight {
        HOME,
        SCORE_L4,
        SCORE_L3,
        SCORE_L2,
        SCORE_L1,
        SHALLOW_PREP,
        SHALLOW_CLIMB;
    }

   
    private TreeMap<ElevatorHeight, Double> elevatorPositions = new TreeMap<ElevatorHeight, Double>();
                
    private ElevatorHeight desiredPosition = ElevatorHeight.HOME;
    DigitalInput limitSwitchHome = new DigitalInput(ElevatorCal.ELEVATOR_LIMIT_SWITCH_DIO_1);
    DigitalInput limitSwitchBelowHome = new DigitalInput(ElevatorCal.ELEVATOR_LIMIT_SWITCH_DIO_2);
    DigitalInput limitSwitchTop = new DigitalInput(ElevatorCal.ELEVATOR_LIMIT_SWITCH_DIO_3);
    private TalonFX leftMotor = new TalonFX(RobotMap.LEFT_ELEVATOR_MOTOR_CAN_ID);
    private TalonFX rightMotor = new TalonFX(RobotMap.RIGHT_ELEVATOR_MOTOR_CAN_ID);
   
    private final TrapezoidProfile m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(ElevatorCal.MAX_VELOCITY_IN_PER_SECOND_SCORE, ElevatorCal.MAX_ACCELERATION_IN_PER_SECOND_SQUARED_SCORE)
    );
    
    private boolean isScoring  =  false;
    private int currentSlotValue = 0;

    public Elevator() {

        elevatorPositions.put(ElevatorHeight.HOME, ElevatorCal.POSITION_HOME_INCHES);
        elevatorPositions.put(ElevatorHeight.SCORE_L4, ElevatorCal.POSITION_SCORE_L4_INCHES);
        elevatorPositions.put(ElevatorHeight.SCORE_L3, ElevatorCal.POSITION_SCORE_L3_INCHES);
        elevatorPositions.put(ElevatorHeight.SCORE_L2, ElevatorCal.POSITION_SCORE_L2_INCHES);
        elevatorPositions.put(ElevatorHeight.SCORE_L1, ElevatorCal.POSITION_SCORE_L1_INCHES);
        elevatorPositions.put(ElevatorHeight.SHALLOW_PREP, ElevatorCal.POSITION_PRE_SHALLOWCLIMB_INCHES);
        elevatorPositions.put(ElevatorHeight.SHALLOW_CLIMB, ElevatorCal.POSITION_SCORE_SHALLOWCLIMB_INCHES);
        initTalons();
    }

    private void initTalons() {
        TalonFXConfigurator cfgLeft = leftMotor.getConfigurator();
        TalonFXConfiguration toApply = new TalonFXConfiguration();

        toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                                               
        toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        toApply.CurrentLimits.SupplyCurrentLimit = ElevatorCal.ELEVATOR_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS;
        toApply.CurrentLimits.StatorCurrentLimit = ElevatorCal.ELEVATOR_MOTOR_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        toApply.CurrentLimits.StatorCurrentLimitEnable = true;
        // Slot 0 is for Scoring PID values and Slot 1 is for Climbing PID values
        toApply.Slot0.kP = ElevatorCal.ELEVATOR_MOTOR_P;
        toApply.Slot0.kI = ElevatorCal.ELEVATOR_MOTOR_I;
        toApply.Slot0.kD = ElevatorCal.ELEVATOR_MOTOR_D;
        toApply.Slot0.kV = ElevatorCal.ELEVATOR_MOTOR_FF;
        toApply.Slot1.kP = ElevatorCal.ELEVATOR_CLIMB_P;
        toApply.Slot1.kI = ElevatorCal.ELEVATOR_CLIMB_I;
        toApply.Slot1.kD = ElevatorCal.ELEVATOR_CLIMB_D;
        toApply.Slot1.kV = ElevatorCal.ELEVATOR_CLIMB_FF;
        cfgLeft.apply(toApply);
        Follower master = new Follower(leftMotor.getDeviceID(), true);
        rightMotor.setControl(master);

    }

    public void setDesiredPosition(ElevatorHeight height) {
        desiredPosition = height;
    }

    public void setControlParams(boolean isScoring) {
        this.isScoring = isScoring;
        if (isScoring) {
            currentSlotValue = 0; // Sets PID values to score
        } else{
            currentSlotValue = 1; // sets PID values to climb
        }
        

    }
 
    private void controlPosition(double inputPositionInch) {
        TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
        TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
        final PositionVoltage m_request = new PositionVoltage(0.0).withSlot(currentSlotValue);
        double rotations = inputPositionInch / ElevatorConstants.GER_CIRCUFRINCE;      
        m_goal = new TrapezoidProfile.State(rotations, 0.0);
        
        

        m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal);

        m_request.Position = m_setpoint.position;
        m_request.Velocity = m_setpoint.velocity;

        leftMotor.setControl(m_request);
   }

   public boolean getLimitSwitchHome(){
        return limitSwitchHome.get();
    }

    public boolean getLimitSwitchBelowHome(){
        return limitSwitchBelowHome.get();
    }

    public boolean getLimitSwitchTop(){
        return limitSwitchTop.get();
    }

   public void periodic() {
        controlPosition(elevatorPositions.get(desiredPosition));

   }

   @Override
   public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addIntegerProperty(
            "Currrent slot value", () -> currentSlotValue, null);
        builder.addDoubleProperty(
            "Left Motor Speed", () -> leftMotor.get(), null);
        builder.addDoubleProperty(
            "Right Motor Speed", () -> rightMotor.get(), null);
        builder.addBooleanProperty(
            "Boolean isClimbig", () -> isScoring, null);
        builder.addDoubleProperty(
            "Desired Position", () -> elevatorPositions.get(desiredPosition), null);
        builder.addDoubleProperty(
            "Left Motor Position", () -> (leftMotor.getPosition().getValueAsDouble() * ElevatorConstants.GER_CIRCUFRINCE * ElevatorConstants.GEAR_RATIO), null);
        builder.addDoubleProperty(
            "Right Motor Position", () -> (rightMotor.getPosition().getValueAsDouble() * ElevatorConstants.GER_CIRCUFRINCE * ElevatorConstants.GEAR_RATIO), null);
        
        
        

   }
}
