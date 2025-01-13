package frc.robot.subsystems.arm;

import java.util.TreeMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
    public final TalonFX armMotorLeft = new TalonFX(RobotMap.ARM_MOTOR_A_CAN_ID);
    public final TalonFX armMotorRight = new TalonFX(RobotMap.ARM_MOTOR_B_CAN_ID);
    private final Follower follower = new Follower(armMotorRight.getDeviceID(), true);

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

    private final ProfiledPIDController armPIDController = new ProfiledPIDController(
            ArmCal.ARM_MOTOR_P, ArmCal.ARM_MOTOR_I, ArmCal.ARM_MOTOR_D,
            new TrapezoidProfile.Constraints(ArmCal.ARM_MAX_VELOCITY_MPS, ArmCal.ARM_MAX_ACCELERATION_MPS_SQUARED));

    public Arm() {
        armPositions.put(ArmPosition.HOME, ArmCal.ARM_POSITION_HOME_DEGREES);
        armPositions.put(ArmPosition.INTAKE, ArmCal.ARM_POSITION_L2_DEGREES);
        armPositions.put(ArmPosition.L2, ArmCal.ARM_POSITION_L3_DEGREES);
        armPositions.put(ArmPosition.L3, ArmCal.ARM_POSITION_L4_DEGREES);
        armPositions.put(ArmPosition.L4, ArmCal.ARM_POSITION_INTAKE_DEGREES);

        initArmTalons();
    }
    
    private void initArmTalons() {
        TalonFXConfigurator cfgMotorRight = armMotorRight.getConfigurator();
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

        cfgMotorRight.apply(toApply);
        armMotorLeft.setControl(follower);
    }

    public void setDesiredPosition(ArmPosition armPosition) {
        this.armDesiredPosition = armPosition;
        this.allowArmMovement = true;
    }

    public void rezeroArm() {
        armMotorRight.setPosition(armRightEncoderAbs.getAbsolutePosition().getValueAsDouble() * 360);
    }

    // Account for PID when setting position of our arm
    public void controlPosition(double inputPositionDegrees) {
        armPIDController.setGoal(inputPositionDegrees);
        armDemandVolts = armPIDController.calculate(armMotorRight.getPosition().getValueAsDouble());

        this.desiredSetpointPosition = armPIDController.getSetpoint().position;
        this.desiredSetpointVelocity = armPIDController.getSetpoint().velocity;

        armMotorRight.setVoltage(armDemandVolts);
    }
    public boolean atPosition(ArmPosition pos) {
        double checkPositionDeg = armPositions.get(pos);
        double currentPositionDeg = armMotorRight.getPosition().getValueAsDouble();

        return Math.abs(checkPositionDeg - currentPositionDeg) < ArmCal.ARM_MARGIN_DEGREES;
    }
    /** TODO: Add interference zone checks w/ Deep Climb */


    public void stopArmMovement() {
        // left motor follows right motor, so armMotorLeft is not necessary here
        armMotorRight.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        // control arm position
        if (allowArmMovement) {
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
        builder.addDoubleProperty("Motor Right Angle (Relative) ", (() -> armMotorRight.getPosition().getValueAsDouble()), null);
        builder.addDoubleProperty("Motor Left Angle (Relative) ", (() -> armMotorLeft.getPosition().getValueAsDouble()), null);
        builder.addDoubleProperty("Motor Right Angle (Absolute) ", (() -> armRightEncoderAbs.getAbsolutePosition().getValueAsDouble()), null);
    }
}