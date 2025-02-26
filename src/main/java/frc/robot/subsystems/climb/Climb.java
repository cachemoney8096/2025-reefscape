package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import java.util.TreeMap;

public class Climb extends SubsystemBase {
  private final TalonFX climbTalonLeft = new TalonFX(RobotMap.CLIMBING_LEFT_MOTOR_CAN_ID);
  private final TalonFX climbTalonRight = new TalonFX(RobotMap.CLIMBING_RIGHT_MOTOR_CAN_ID);
  // private final CANcoder climbAbsoluteEncoder = new CANcoder(RobotMap.CLIMB_ABS_ENCODER_CAN_ID);
  private final Encoder climbAbsoluteEncoder =
      new Encoder(RobotMap.CLIMBING_ABS_ENCODER_DIO_A, RobotMap.CLIMBING_ABS_ENCODER_DIO_B);

  public enum ClimbPosition {
    CLIMBING_PREP,
    CLIMBING,
    STOWED,
  }

  private TreeMap<ClimbPosition, Double> climbPositionMap;
  private ClimbPosition desiredPosition = ClimbPosition.STOWED;
  private boolean allowClimbMovement = false;
  private TrapezoidProfile.State tSetpoint = new TrapezoidProfile.State();
  private final TrapezoidProfile trapezoidProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ClimbCal.CLIMB_MOTOR_MAX_VELOCITY_RPS,
              ClimbCal.CLIMB_MOTOR_MAX_ACCELERATION_RPS_SQUARED));

  private int currentSlot = 1;

  public Climb() {
    initClimbTalons();
    climbAbsoluteEncoder.setDistancePerPulse(
        Constants.DEGREES_PER_REV_THROUGH_BORE_ABS_ENCODER_PULSE);
    climbPositionMap = new TreeMap<ClimbPosition, Double>();
    climbPositionMap.put(ClimbPosition.CLIMBING, ClimbCal.CLIMB_CLIMBING_POSITION_DEGREES);
    climbPositionMap.put(ClimbPosition.STOWED, ClimbCal.CLIMB_STOWED_POSITION_DEGREES);
    climbPositionMap.put(ClimbPosition.CLIMBING_PREP, ClimbCal.CLIMB_CLIMBING_PREP_DEGREES);
  }

  private void initClimbTalons() {
    TalonFXConfigurator cfgLeft = climbTalonLeft.getConfigurator();

    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.CurrentLimits.SupplyCurrentLimit = ClimbCal.CLIMB_TALONS_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimit = ClimbCal.CLIMB_TALONS_STATOR_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.SupplyCurrentLimitEnable = true;
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    toApply.Slot0.kP = ClimbCal.CLIMBING_P;
    toApply.Slot0.kI = ClimbCal.CLIMBING_I;
    toApply.Slot0.kD = ClimbCal.CLIMBING_D;
    toApply.Slot0.kV = ClimbCal.CLIMBING_FF;

    toApply.Slot1.kP = ClimbCal.POSITIONING_P;
    toApply.Slot1.kI = ClimbCal.POSITIONING_I;
    toApply.Slot1.kD = ClimbCal.POSITIONING_D;
    toApply.Slot1.kV = ClimbCal.POSITIONING_FF;

    cfgLeft.apply(toApply);

    climbTalonLeft.getDutyCycle().setUpdateFrequency(ClimbCal.CLIMB_DUTY_CYCLE_UPDATE_FREQ_HZ);

    Follower master = new Follower(climbTalonLeft.getDeviceID(), true);
    climbTalonRight.setControl(master);
  }

  public void zeroMotorEncoders() {
    // climbTalonLeft.setPosition(climbAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
    climbTalonLeft.setPosition(climbAbsoluteEncoder.getDistance());
    tSetpoint =
        new TrapezoidProfile.State(
            // climbAbsoluteEncoder.getAbsolutePosition().getValueAsDouble(), 0.0);
            climbAbsoluteEncoder.getDistance(), 0.0);
  }

  public void setClimbingPID() {
    currentSlot = 0;
  }

  public void setPositioningPID() {
    currentSlot = 1;
  }

  private void controlPosition(double inputPositionDegrees) {
    TrapezoidProfile.State tGoal =
        new TrapezoidProfile.State(
            inputPositionDegrees / 360.0,
            0);

    PositionVoltage tRequest = new PositionVoltage(0.0).withSlot(currentSlot);
    // set next setpoint, where t = periodic interval (20ms)
    tSetpoint = trapezoidProfile.calculate(Constants.PERIOD_TIME_SECONDS, tSetpoint, tGoal);

    tRequest.Position = tSetpoint.position;
    tRequest.Velocity = tSetpoint.velocity;

    climbTalonLeft.setControl(tRequest);
  }

  public void setDesiredClimbPosition(ClimbPosition pos) {
    this.desiredPosition = pos;
    this.allowClimbMovement = true;
  }

  public void doNotAllowClimbMovement() {
    this.allowClimbMovement = false;
  }

  public boolean atClimbPosition(ClimbPosition checkPos) {
    double currentPosition = climbTalonRight.getPosition().getValueAsDouble() * 360.0;
    double checkPosDegrees = climbPositionMap.get(checkPos);

    return Math.abs(currentPosition - checkPosDegrees) <= ClimbCal.CLIMB_MARGIN_DEGREES;
  }

  public boolean atDesiredPosition() {
    return atClimbPosition(desiredPosition);
  }

  public void stopClimbMovement() {
    climbTalonLeft.setVoltage(0.0);
  }

  public void testClimbMovementUp() {
    climbTalonLeft.setVoltage(ClimbCal.TEST_CLIMB_MOVEMENT_VOLTAGE);
  }

  public void testClimbMovementDown() {
    climbTalonLeft.setVoltage(-1 * ClimbCal.TEST_CLIMB_MOVEMENT_VOLTAGE);
  }

  @Override
  public void periodic() {
    if (allowClimbMovement) {
      controlPosition(climbPositionMap.get(this.desiredPosition));
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Desired Position", () -> desiredPosition.toString(), null);
    builder.addBooleanProperty("At Desired Position", this::atDesiredPosition, null);
    builder.addDoubleProperty(
        "Desired Position (Deg)", () -> climbPositionMap.get(desiredPosition), null);
    builder.addDoubleProperty(
        "Current Left Motor Position (Deg)",
        () -> climbTalonLeft.getPosition().getValueAsDouble() * 360.0,
        null);
    builder.addDoubleProperty(
        "Current Right Motor Position (Deg)",
        () -> climbTalonRight.getPosition().getValueAsDouble() * 360.0,
        null);
    builder.addBooleanProperty("Allow Climb Movement", () -> allowClimbMovement, null);
    builder.addDoubleProperty(
        "Absolute Encoder Distance (Deg)", () -> climbAbsoluteEncoder.getDistance() * 360, null);
    builder.addDoubleProperty("Current Slot", () -> currentSlot, null);
    builder.addDoubleProperty("Trapezoid Setpoint Position (revs)", () -> tSetpoint.position, null);
    builder.addDoubleProperty(
        "Trapezoid Setpoint Velocity (revs/sec)", () -> tSetpoint.velocity, null);
  }
}
