package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import java.util.TreeMap;

public class Elevator extends SubsystemBase {
  public enum ElevatorHeight {
    HOME,
    INTAKE,
    SCORE_L4,
    SCORE_L3,
    SCORE_L2,
    SCORE_L1,
    SHALLOW_PREP,
    SHALLOW_CLIMB,
    ARM_CLEAR_OF_CLIMB;
  }

  private TreeMap<ElevatorHeight, Double> elevatorPositions = new TreeMap<ElevatorHeight, Double>();

  private ElevatorHeight desiredPosition = ElevatorHeight.HOME;
  DigitalInput limitSwitchHome = new DigitalInput(ElevatorCal.ELEVATOR_LIMIT_SWITCH_DIO_HOME);
  DigitalInput limitSwitchBelowHome =
      new DigitalInput(ElevatorCal.ELEVATOR_LIMIT_SWITCH_DIO_BELOWHOME);
  DigitalInput limitSwitchTop = new DigitalInput(ElevatorCal.ELEVATOR_LIMIT_SWITCH_DIO_TOP);
  private TalonFX leftMotor = new TalonFX(RobotMap.LEFT_ELEVATOR_MOTOR_CAN_ID);
  private TalonFX rightMotor = new TalonFX(RobotMap.RIGHT_ELEVATOR_MOTOR_CAN_ID);
  ;

  private final TrapezoidProfile m_profile_scoring =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ElevatorCal.MAX_VELOCITY_IN_PER_SECOND_SCORE,
              ElevatorCal.MAX_ACCELERATION_IN_PER_SECOND_SQUARED_SCORE));
  private final TrapezoidProfile m_profile_climbing =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ElevatorCal.MAX_VELOCITY_IN_PER_SECOND_CLIMB,
              ElevatorCal.MAX_ACCELERATION_IN_PER_SECOND_SQUARED_CLIMB));

  private boolean isScoring = true;
  private int currentSlotValue = 0; // 0 is for scoring and 1 is for climbing

  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

  private boolean allowElevatorMovement = false;

  public Elevator() {

    elevatorPositions.put(ElevatorHeight.HOME, ElevatorCal.POSITION_HOME_INCHES);
    elevatorPositions.put(ElevatorHeight.INTAKE, ElevatorCal.POSITION_INTAKE_INCHES);
    elevatorPositions.put(ElevatorHeight.SCORE_L4, ElevatorCal.POSITION_SCORE_L4_INCHES);
    elevatorPositions.put(ElevatorHeight.SCORE_L3, ElevatorCal.POSITION_SCORE_L3_INCHES);
    elevatorPositions.put(ElevatorHeight.SCORE_L2, ElevatorCal.POSITION_SCORE_L2_INCHES);
    elevatorPositions.put(ElevatorHeight.SCORE_L1, ElevatorCal.POSITION_SCORE_L1_INCHES);
    elevatorPositions.put(ElevatorHeight.SHALLOW_PREP, ElevatorCal.POSITION_SHALLOW_PREP_INCHES);
    elevatorPositions.put(ElevatorHeight.SHALLOW_CLIMB, ElevatorCal.POSITION_SHALLOW_CLIMB_INCHES);
    elevatorPositions.put(
        ElevatorHeight.ARM_CLEAR_OF_CLIMB, ElevatorCal.POSITION_ARM_CLEAR_OF_CLIMB_INCHES);
    initTalons();
  }

  private void initTalons() {
    TalonFXConfigurator cfgLeft = leftMotor.getConfigurator();
    TalonFXConfiguration toApply = new TalonFXConfiguration();

    // TODO change this accordingly
    toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    toApply.CurrentLimits.SupplyCurrentLimit = ElevatorCal.ELEVATOR_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimit =
        ElevatorCal.ELEVATOR_MOTOR_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimitEnable = true;
    // Slot 0 is for Scoring PID values and Slot 1 is for Shallow Climbing PID values
    toApply.Slot0.kP = ElevatorCal.ELEVATOR_SCORE_P;
    toApply.Slot0.kI = ElevatorCal.ELEVATOR_SCORE_I;
    toApply.Slot0.kD = ElevatorCal.ELEVATOR_SCORE_D;
    toApply.Slot0.kV = ElevatorCal.ELEVATOR_SCORE_FF;
    
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
    currentSlotValue = isScoring ? 0 : 1; // 0 for scoring, 1 for climbing
  }

  private void controlPosition(double inputPositionInch) {
    final PositionVoltage m_request = new PositionVoltage(0.0).withSlot(currentSlotValue);
    double rotations =
        inputPositionInch
            / ElevatorConstants.DRUM_CIRCUMFERENCE
            * ElevatorConstants.MOTOR_TO_DRUM_RATIO;
    m_goal = new TrapezoidProfile.State(rotations, 0.0);

    m_setpoint =
        (isScoring ? m_profile_scoring : m_profile_climbing).calculate(0.02, m_setpoint, m_goal);

    m_request.Position = m_setpoint.position;
    m_request.Velocity = m_setpoint.velocity;

    leftMotor.setControl(m_request);
  }

  public boolean atDesiredPosition() {
    return Math.abs(
            leftMotor.getPosition().getValueAsDouble()
                - elevatorPositions.get(desiredPosition)
                    / ElevatorConstants.DRUM_CIRCUMFERENCE
                    * ElevatorConstants.MOTOR_TO_DRUM_RATIO)
        < ElevatorCal.ELEVATOR_MARGIN_DEGREES;
  }

  public boolean atElevatorPosition(ElevatorHeight height) {
    return Math.abs(
            leftMotor.getPosition().getValueAsDouble()
                - elevatorPositions.get(height)
                    / ElevatorConstants.DRUM_CIRCUMFERENCE
                    * ElevatorConstants.MOTOR_TO_DRUM_RATIO)
        < ElevatorCal.ELEVATOR_MARGIN_DEGREES;
  }

  public boolean armMovementAllowed() {
    return leftMotor.getPosition().getValueAsDouble()
        > (elevatorPositions.get(ElevatorHeight.ARM_CLEAR_OF_CLIMB)
                - ElevatorCal.AT_CLEAR_POSITION_MARGIN)
            / ElevatorConstants.DRUM_CIRCUMFERENCE
            * ElevatorConstants.MOTOR_TO_DRUM_RATIO; // someone reviewing my work check this please
  }

  /* The limit switches we are using are active low, hence the ! operator */
  public boolean getLimitSwitchHome() {
    return !limitSwitchHome.get();
  }

  public boolean getLimitSwitchBelowHome() {
    return !limitSwitchBelowHome.get();
  }

  public boolean getLimitSwitchTop() {
    return !limitSwitchTop.get();
  }

  public void periodic() {
    if (allowElevatorMovement) {
      controlPosition(elevatorPositions.get(desiredPosition));
    }
  }

  public void stopElevatorMovement() {
    leftMotor.setVoltage(0.0);
  }

  public void testElevatorMovementUp() {
    leftMotor.setVoltage(ElevatorCal.TEST_ELEVATOR_MOVEMENT_VOLTAGE);
  }

  public void testElevatorMovementDown() {
    leftMotor.setVoltage(-1 * ElevatorCal.TEST_ELEVATOR_MOVEMENT_VOLTAGE);
  }

  public void zeroElevatorToHome() {
    leftMotor.setPosition(
        ElevatorCal.POSITION_HOME_INCHES
            / ElevatorConstants.DRUM_CIRCUMFERENCE
            * ElevatorConstants.MOTOR_TO_DRUM_RATIO);
    m_setpoint =
        new TrapezoidProfile.State(
            ElevatorCal.POSITION_HOME_INCHES
                / ElevatorConstants.DRUM_CIRCUMFERENCE
                * ElevatorConstants.MOTOR_TO_DRUM_RATIO,
            0.0);
  }

  public void setElevatorMovementAllowed(boolean allowed) {
    allowElevatorMovement = allowed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("Boolean isClimbing", () -> !isScoring, null);
    builder.addIntegerProperty("Currrent slot value", () -> currentSlotValue, null);
    builder.addStringProperty("Desired Position", () -> desiredPosition.toString(), null);
    builder.addBooleanProperty("Limit Switch Home ", () -> getLimitSwitchHome(), null);
    builder.addBooleanProperty("Limit Switch Below Home", () -> getLimitSwitchBelowHome(), null);
    builder.addBooleanProperty("Limit Switch Switch Top ", () -> getLimitSwitchTop(), null);
    builder.addDoubleProperty(
        "Current Left Motor Position (Deg)",
        () -> leftMotor.getPosition().getValueAsDouble() * 360.0,
        null);
    builder.addDoubleProperty(
        "Current Right Motor Position (Deg)",
        () -> rightMotor.getPosition().getValueAsDouble() * 360.0,
        null);
    builder.addBooleanProperty("Allow Elevator Movement", () -> allowElevatorMovement, null);
    builder.addDoubleProperty(
        "Trapezoid Setpoint Position (revs)", () -> m_setpoint.position, null);
    builder.addDoubleProperty(
        "Trapezoid Setpoint Velocity (revs/sec)", () -> m_setpoint.velocity, null);

    builder.addDoubleProperty(
        "Desired position inches", () -> elevatorPositions.get(desiredPosition), null);
    builder.addBooleanProperty("At desired position ", this::atDesiredPosition, null);
    builder.addDoubleProperty(
        "Current position",
        () ->
            (leftMotor.getPosition().getValueAsDouble()
                * ElevatorConstants.DRUM_CIRCUMFERENCE
                / ElevatorConstants.MOTOR_TO_DRUM_RATIO),
        null);
  }
}
