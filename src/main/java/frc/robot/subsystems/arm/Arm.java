package frc.robot.subsystems.arm;

import java.util.TreeMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
    public final TalonFX armMotorLeft = new TalonFX(RobotMap.ARM_MOTOR_LEFT_CAN_ID);
    public final TalonFX armMotorRight = new TalonFX(RobotMap.ARM_MOTOR_RIGHT_CAN_ID);

    private final CANcoder armRightEncoderAbs = new CANcoder(RobotMap.ARM_ABS_ENCODER_CAN_ID);
    public enum ArmPosition {
        HOME,
        INTAKE,
        L2,
        L3,
        L4,
    }

    private double armDemandVolts = 0.0;
    private boolean allowArmMovement = false;
    private double desiredSetpointPosition = 0.0;
    private double desiredSetpointVelocity = 0.0;

    /**
     * Map each of our arm positions to an actual position on our arm (degrees)
     */
    public final TreeMap<ArmPosition, Double> armPositions = new TreeMap<ArmPosition, Double>();

    private ArmPosition armDesiredPosition = ArmPosition.HOME;
    public Arm() {
        armPositions.put(ArmPosition.HOME, ArmCal.ARM_POSITION_HOME_DEGREES);
        armPositions.put(ArmPosition.INTAKE, ArmCal.ARM_POSITION_L2_DEGREES);
        armPositions.put(ArmPosition.L2, ArmCal.ARM_POSITION_L3_DEGREES);
        armPositions.put(ArmPosition.L3, ArmCal.ARM_POSITION_L4_DEGREES);
        armPositions.put(ArmPosition.L4, ArmCal.ARM_POSITION_INTAKE_DEGREES);

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

        armMotorRight.getConfigurator().apply(toApply);
        armMotorLeft.setControl(new Follower(armMotorRight.getDeviceID(), true));
    }

    public void setDesiredPosition(ArmPosition armPosition) {
        /**
         * If arm is within interference zone, don't set desired position
         */
        if (isArmMoveable()) {
            this.armDesiredPosition = armPosition;
        }
    }

    public void setAllowArmMovement(boolean armMovement) {
        this.allowArmMovement = armMovement;
    }
    public boolean isArmMoveable() {
        return this.allowArmMovement;
    }

    public void rezeroArm() {
        armMotorRight.setPosition(armRightEncoderAbs.getAbsolutePosition().getValueAsDouble() * 360);
    }

    // Account for PID when setting position of our arm
    public void controlPosition(double inputPositionDegrees) {
        PositionVoltage desiredVoltage = new PositionVoltage(inputPositionDegrees).withSlot(0);
        armMotorRight.setControl(desiredVoltage);
        // desired pos = requested pos
        this.desiredSetpointPosition = inputPositionDegrees;
        // convert velocity in rotations/sec to deg/sec
        this.desiredSetpointVelocity = armMotorRight.getVelocity().getValueAsDouble() * 360;
    }
    // TODO: determine whether the right or left motor is the one that should be matching degree position
    public boolean atPositionDegrees(ArmPosition pos) {
        double checkPositionDeg = armPositions.get(pos);
        double currentPositionDeg = armMotorRight.getPosition().getValueAsDouble() * 360;

        return Math.abs(checkPositionDeg - currentPositionDeg) < ArmCal.ARM_MARGIN_DEGREES;
    }
    /** TODO: Implement interference zone checking functionality */
    public boolean isArmInInterferenceZone() {
        return Constants.PLACEHOLDER_BOOLEAN;
    }
    

    public void stopArmMovement() {
        // left motor follows right motor, so armMotorLeft is not necessary here
        armMotorRight.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        setAllowArmMovement(isArmInInterferenceZone());
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
        builder.addBooleanProperty("Is Arm Movement Allowed", (() -> allowArmMovement), null);
        builder.addDoubleProperty("Motor Right Angle (Relative) ", (() -> armMotorRight.getPosition().getValueAsDouble() * 360), null);
        builder.addDoubleProperty("Motor Left Angle (Relative) ", (() -> armMotorLeft.getPosition().getValueAsDouble() * 360), null);
        builder.addDoubleProperty("Motor Right Angle (Absolute) ", (() -> armRightEncoderAbs.getAbsolutePosition().getValueAsDouble() * 360), null);
    }
}