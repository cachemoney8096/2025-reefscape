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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
    public enum ElevatorHeight {
        HOME,
        SCORE_L4,
        SCORE_L3,
        SCORE_L2,
        SCORE_L1,
        PRE_SHALLOWCLIMB,
        SCORE_SHALLOWCLIMB;
    }

   
    private TreeMap<ElevatorHeight, Double> elevatorPositions = new TreeMap<ElevatorHeight, Double>();
                
    private ElevatorHeight desiredPosition = ElevatorHeight.HOME;

    public TalonFX leftMotor = new TalonFX(RobotMap.LEFT_ELEVATOR_CAN_ID);
    public TalonFX rightMotor = new TalonFX(RobotMap.RIGHT_ELEVATOR_CAN_ID);
   
    final TrapezoidProfile m_profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(ElevatorCal.MAX_VELOCITY_IN_PER_SECOND, ElevatorCal.MAX_ACCELERATION_IN_PER_SECOND_SQUARED)
    );
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

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
        var slot0Configs = new Slot0Configs();
        TalonFXConfigurator cfgLeft = leftMotor.getConfigurator();
        TalonFXConfigurator cfgRight = rightMotor.getConfigurator();
        TalonFXConfiguration toApply = new TalonFXConfiguration();

        toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                                               
        toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast; 
        toApply.CurrentLimits.SupplyCurrentLimit = ElevatorCal.ELEVATOR_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS;
        toApply.CurrentLimits.StatorCurrentLimit = ElevatorCal.ELEVATOR_MOTOR_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        toApply.CurrentLimits.StatorCurrentLimitEnable = true;
        slot0Configs.kP = ElevatorCal.ELEVATOR_MOTOR_P;
        slot0Configs.kI = ElevatorCal.ELEVATOR_MOTOR_I;
        slot0Configs.kD = ElevatorCal.ELEVATOR_MOTOR_D;
        slot0Configs.kV = ElevatorCal.ELEVATOR_MOTOR_FF;
        cfgLeft.apply(toApply);
        cfgRight.apply(toApply);
        Follower master = new Follower(leftMotor.getDeviceID(), true); //TODO we dont know if opposite master direction is true yet
        rightMotor.setControl(master);
        rightMotor.getConfigurator().apply(slot0Configs);

    }

    public void setDesiredPosition(ElevatorHeight height){
        desiredPosition = height;

    }

    private void controlPosition(double inputPositionInch) {
        m_goal = new TrapezoidProfile.State(inputPositionInch, 0);
        m_setpoint = m_profile.calculate(ElevatorCal.ELEVATOR_MOTOR_D, m_setpoint, m_goal);
        
        final PositionVoltage m_request = new PositionVoltage(inputPositionInch).withSlot(0);

        m_setpoint = m_profile.calculate(ElevatorCal.ELEVATOR_MOTOR_D, m_setpoint, m_goal);

        m_request.Position = m_setpoint.position;
        m_request.Velocity = m_setpoint.velocity;

        leftMotor.setControl(m_request);
   }

   public void periodic() {
        controlPosition(elevatorPositions.get(desiredPosition));

   }
}