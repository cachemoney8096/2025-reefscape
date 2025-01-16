package frc.robot.subsystems.arm;

import java.util.TreeMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
    public final TalonFX armMotorLeft = new TalonFX(RobotMap.ARM_MOTOR_LEFT_CAN_ID);
    public final TalonFX armMotorRight = new TalonFX(RobotMap.ARM_MOTOR_RIGHT_CAN_ID);

    private final CANcoder armLefttEncoderAbs = new CANcoder(RobotMap.ARM_ABS_ENCODER_CAN_ID);
    public enum ArmPosition {
        // home will also be the position for when we are deep climbing
        HOME,
        INTAKE,
        L2,
        L3,
        L4,
    }

    private double armDemandVolts = 0.0;
    private double desiredSetpointPosition = 0.0;
    private double desiredSetpointVelocity = 0.0;

    /**
     * Map each of our arm positions to an actual position on our arm (degrees)
     */
    public final TreeMap<ArmPosition, Double> armPositions = new TreeMap<ArmPosition, Double>();

    private ArmPosition armDesiredPosition = ArmPosition.HOME;
    public Arm() {
        armPositions.put(ArmPosition.HOME, ArmCal.ARM_POSITION_HOME_DEGREES);
        armPositions.put(ArmPosition.INTAKE, ArmCal.ARM_POSITION_INTAKE_DEGREES);
        armPositions.put(ArmPosition.L2, ArmCal.ARM_POSITION_L2_DEGREES);
        armPositions.put(ArmPosition.L3, ArmCal.ARM_POSITION_L3_DEGREES);
        armPositions.put(ArmPosition.L4, ArmCal.ARM_POSITION_L4_DEGREES);

        initArmTalons();
    }
    
    private void initArmTalons() {
        TalonFXConfiguration toApply = new TalonFXConfiguration();

        toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        toApply.CurrentLimits.SupplyCurrentLimit = ArmCal.ARM_SUPPLY_CURRENT_LIMIT_AMPS;
        toApply.CurrentLimits.StatorCurrentLimit = ArmCal.ARM_STATOR_CURRENT_LIMIT_AMPS;
        toApply.CurrentLimits.StatorCurrentLimitEnable = true;
        toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        toApply.Slot0.kP = ArmCal.ARM_MOTOR_P;
        toApply.Slot0.kI = ArmCal.ARM_MOTOR_I;
        toApply.Slot0.kD = ArmCal.ARM_MOTOR_D;
        toApply.Slot0.kV = ArmCal.ARM_MOTOR_FF;

        armMotorLeft.getConfigurator().apply(toApply);
        armMotorRight.setControl(new Follower(armMotorLeft.getDeviceID(), true));
    }

    public void setDesiredPosition(ArmPosition armPosition) {
        // If arm is within interference zone, don't set desired position
        if (isArmMoveable()) {
            this.armDesiredPosition = armPosition;
        }
    }

    public boolean isArmMoveable() {
        // return opposite of whether or not arm is in interference zone
        return !isArmInInterferenceZone();
    }

    public void rezeroArm() {
        armMotorLeft.setPosition(armLefttEncoderAbs.getAbsolutePosition().getValueAsDouble() * 360);
    }

    // Account for PID when setting position of our arm
    public void controlPosition(double inputPositionDegrees) {
        // trapezoidal motion profiling to account for large jumps in velocity which result in large error
        final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ArmCal.ARM_MOTOR_MAX_VELOCITY_DPS, ArmCal.ARM_MOTOR_MAX_ACCERLATION_DPS_SQUARED));
        // goal position (rotations) w/ velocity at position (0?)

        // TODO: figure out if our velocity at given position is always going to be 0 or not
        TrapezoidProfile.State tGoal = new TrapezoidProfile.State(inputPositionDegrees / 360, 0);
        TrapezoidProfile.State tSetpoint = new TrapezoidProfile.State();

        PositionVoltage tRequest = new PositionVoltage(0).withSlot(0);
        // set next setpoint, where t = periodic interval (20ms)
        tSetpoint = trapezoidProfile.calculate(0.02, tSetpoint, tGoal);

        tRequest.Position = tSetpoint.position;
        tRequest.Velocity = tSetpoint.velocity;

        armMotorRight.setControl(tRequest);
        // desired pos = requested pos
        this.desiredSetpointPosition = tRequest.Position;
        // convert velocity in rotations/sec to deg/sec
        this.desiredSetpointVelocity = tRequest.Velocity * 360;
    }

    public boolean atPositionDegrees(ArmPosition pos) {
        double checkPositionDeg = armPositions.get(pos);
        double currentPositionDeg = armMotorLeft.getPosition().getValueAsDouble() * 360;

        return Math.abs(checkPositionDeg - currentPositionDeg) < ArmCal.ARM_MARGIN_DEGREES;
    }
    public boolean isArmInInterferenceZone() {
        double currentPosition = armMotorLeft.getPosition().getValueAsDouble() * 360;
        if (currentPosition <= ArmCal.ARM_INTERFERENCE_THRESHOLD_MAX_DEGREES && 
            currentPosition >= ArmCal.ARM_INTERFERENCE_THRESHOLD_MIN_DEGREES 
            /** TODO: AND is Deep Climb engaged */ ) {
            return true;
        }
        return false;
    }
    

    public void stopArmMovement() {
        // left motor follows right motor, so armMotorLeft is not necessary here
        armMotorLeft.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        // control arm position
        if (isArmMoveable()) {
            controlPosition(armPositions.get(this.armDesiredPosition));
        }
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Desired Setpoint Position (Deg)", (() -> desiredSetpointPosition), null);
        builder.addDoubleProperty("Current Position (Relative Encoder) (Deg)", (() -> armMotorRight.getPosition().getValueAsDouble() * 360), null);
        builder.addDoubleProperty("Desired Setpoint Velocity (m/s)", (() -> desiredSetpointVelocity), null);
        builder.addDoubleProperty("Arm Demand Volts (calc)", (() -> armDemandVolts), null);
        builder.addBooleanProperty("Is Arm In Interference Zone", this::isArmInInterferenceZone, null);
        builder.addDoubleProperty("Motor Right Angle (Relative) ", (() -> armMotorRight.getPosition().getValueAsDouble() * 360), null);
        builder.addDoubleProperty("Motor Left Angle (Relative) ", (() -> armMotorLeft.getPosition().getValueAsDouble() * 360), null);
        builder.addDoubleProperty("Motor Right Angle (Absolute) ", (() -> armLefttEncoderAbs.getAbsolutePosition().getValueAsDouble() * 360), null);
    }
}