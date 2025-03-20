package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.arm.ArmCal;

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
    ARM_CLEAR_OF_CLIMB,
    ALGAE;
  }

  private TreeMap<ElevatorHeight, Double> elevatorPositions = new TreeMap<ElevatorHeight, Double>();

  private ElevatorHeight desiredPosition = ElevatorHeight.HOME;
  /*DigitalInput limitSwitchHome = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH_DIO_HOME);
  DigitalInput limitSwitchBelowHome =
      new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH_DIO_BELOWHOME);
  DigitalInput limitSwitchTop = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH_DIO_TOP);*/
  private TalonFX leftMotor = new TalonFX(RobotMap.LEFT_ELEVATOR_MOTOR_CAN_ID, "rio");
  private TalonFX rightMotor = new TalonFX(RobotMap.RIGHT_ELEVATOR_MOTOR_CAN_ID, "rio");
  private CANrange canrange = new CANrange(RobotMap.ELEVATOR_CANRANGE, "rio");

  private boolean isScoring = true;
  private int currentSlotValue = 0; // 0 is for scoring and 1 is for climbing

  private boolean allowElevatorMovement = true; //TODO

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
    elevatorPositions.put(ElevatorHeight.ALGAE, ElevatorCal.POSITION_ALGAE_INCHES);
    initTalons();

    FovParamsConfigs fovCfg = new FovParamsConfigs();
    fovCfg.FOVCenterX = 0.0;
    fovCfg.FOVCenterY = 0.0;
    fovCfg.FOVRangeX = 2.0;
    fovCfg.FOVRangeY = 2.0;
    canrange.getConfigurator().apply(new CANrangeConfiguration().withFovParams(fovCfg));
  }

  private void initTalons() {
    TalonFXConfigurator cfgLeft = leftMotor.getConfigurator();
    TalonFXConfiguration toApply = new TalonFXConfiguration();

    toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    toApply.CurrentLimits.SupplyCurrentLimit = ElevatorCal.ELEVATOR_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimit =
        ElevatorCal.ELEVATOR_MOTOR_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimitEnable = true;
    // Slot 0 is for Scoring PID values and Slot 1 is for Shallow Climbing PID values
    toApply.Slot0.kP = ElevatorCal.ELEVATOR_SCORE_P;
    toApply.Slot0.kI = ElevatorCal.ELEVATOR_SCORE_I;
    toApply.Slot0.kD = ElevatorCal.ELEVATOR_SCORE_D;
    toApply.Slot0.kV = ElevatorCal.ELEVATOR_SCORE_FF;
    toApply.Slot0.kG = 0.25;

    toApply.Slot1.kP = ElevatorCal.ELEVATOR_CLIMB_P;
    toApply.Slot1.kI = ElevatorCal.ELEVATOR_CLIMB_I;
    toApply.Slot1.kD = ElevatorCal.ELEVATOR_CLIMB_D;
    toApply.Slot1.kV = ElevatorCal.ELEVATOR_CLIMB_FF;
    cfgLeft.apply(toApply);
    Follower master = new Follower(leftMotor.getDeviceID(), true);
    rightMotor.setControl(master);

    zeroElevatorToHome();
    // zeroElevatorUsingCanrange();
  }

  public void setDesiredPosition(ElevatorHeight height) {
    desiredPosition = height;
  }

  public double linearSpeedThrottle() {
    if (this.desiredPosition == ElevatorHeight.SCORE_L2 || desiredPosition == ElevatorHeight.SCORE_L3 || desiredPosition == ElevatorHeight.SCORE_L4 || desiredPosition == ElevatorHeight.INTAKE) {
      return 0.3;
    }
    return 1.0;
  }

  public double angularSpeedThrottle() {
    if (this.desiredPosition == ElevatorHeight.SCORE_L2 || desiredPosition == ElevatorHeight.SCORE_L3 || desiredPosition == ElevatorHeight.SCORE_L4 || desiredPosition == ElevatorHeight.INTAKE) {
      return 0.3;
    }
    return 1.0;
  }

  public void setControlParams(boolean isScoring) {
    this.isScoring = isScoring;
    currentSlotValue = isScoring ? 0 : 1; // 0 for scoring, 1 for climbing
  }

  private void controlPosition(double inputPositionInch) {
    // final PositionVoltage m_request = new PositionVoltage(0.0).withSlot(currentSlotValue);
    double rotations =
        inputPositionInch
            / ElevatorConstants.DRUM_CIRCUMFERENCE
            * ElevatorConstants.MOTOR_TO_DRUM_RATIO;
    /*m_goal = new TrapezoidProfile.State(rotations, 0.0);

    m_setpoint =
        (isScoring ? m_profile_scoring : m_profile_climbing)
            .calculate(Constants.PERIOD_TIME_SECONDS, m_setpoint, m_goal);

    m_request.Position = m_setpoint.position;
    m_request.Velocity = m_setpoint.velocity;

    leftMotor.setControl(m_request);*/
    final TrapezoidProfile trapezoidProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              6000, 6000));
    TrapezoidProfile.State tGoal = new TrapezoidProfile.State(rotations, 0.0);
    TrapezoidProfile.State setpoint =
        new TrapezoidProfile.State(leftMotor.getPosition().getValueAsDouble(), leftMotor.getVelocity().getValueAsDouble());
    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    setpoint = trapezoidProfile.calculate(0.020, setpoint, tGoal);
    request.Position = setpoint.position;
    request.Velocity = setpoint.velocity;
    leftMotor.setControl(request);
  }

  public boolean atDesiredPosition() {
    return Math.abs(
            leftMotor.getPosition().getValueAsDouble()* ElevatorConstants.DRUM_CIRCUMFERENCE
            / ElevatorConstants.MOTOR_TO_DRUM_RATIO
                - elevatorPositions.get(desiredPosition)
                )
        < ElevatorCal.ELEVATOR_MARGIN_INCHES;
  }

  public boolean atElevatorPosition(ElevatorHeight height) {
    return Math.abs(
            leftMotor.getPosition().getValueAsDouble()* ElevatorConstants.DRUM_CIRCUMFERENCE
            / ElevatorConstants.MOTOR_TO_DRUM_RATIO
                - elevatorPositions.get(height)
                )
        < ElevatorCal.ELEVATOR_MARGIN_INCHES;
  }

  public boolean armMovementAllowed() {
    return leftMotor.getPosition().getValueAsDouble()
    * ElevatorConstants.DRUM_CIRCUMFERENCE
                / ElevatorConstants.MOTOR_TO_DRUM_RATIO
        > (elevatorPositions.get(ElevatorHeight.ARM_CLEAR_OF_CLIMB)
                - ElevatorCal.AT_CLEAR_POSITION_MARGIN); 
  }

  /* The limit switches we are using are active low, hence the ! operator
  public boolean getLimitSwitchHome() {
    return !limitSwitchHome.get();
  }

  public boolean getLimitSwitchBelowHome() {
    return !limitSwitchBelowHome.get();
  }

  public boolean getLimitSwitchTop() {
    return !limitSwitchTop.get();
  }*/

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
    // leftMotor.setPosition(
    //     ElevatorCal.POSITION_HOME_INCHES
    //         / ElevatorConstants.DRUM_CIRCUMFERENCE
    //         * ElevatorConstants.MOTOR_TO_DRUM_RATIO);
    leftMotor.setPosition(0.0);
  }

  public void zeroElevatorUsingCanrange() {
    double currentHeightRot =
        Units.metersToInches(canrange.getDistance().getValueAsDouble())
            / ElevatorConstants.DRUM_CIRCUMFERENCE
            * ElevatorConstants.MOTOR_TO_DRUM_RATIO;
    leftMotor.setPosition(currentHeightRot);
  }

  public void setElevatorMovementAllowed(boolean allowed) {
    allowElevatorMovement = allowed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addStringProperty("Elevator DESIRED Pos", () -> desiredPosition.toString(), null);
    builder.addDoubleProperty(
        "Elevator DESIRED Pos (in)", () -> elevatorPositions.get(desiredPosition), null);
    builder.addBooleanProperty("Elevator at desired", ()->atDesiredPosition(), null);

    builder.addDoubleProperty(
        "Elevator Left Motor RELATIVE (deg)",
        () -> leftMotor.getPosition().getValueAsDouble() * 360.0,
        null);
    builder.addDoubleProperty(
        "Elevator Right Motor RELATIVE (deg)",
        () -> rightMotor.getPosition().getValueAsDouble() * 360.0,
        null);

    builder.addDoubleProperty(
        "Elevator CURRENT Pos (in)",
        () ->
            (leftMotor.getPosition().getValueAsDouble()
                * ElevatorConstants.DRUM_CIRCUMFERENCE
                / ElevatorConstants.MOTOR_TO_DRUM_RATIO),
        null);

    builder.addStringProperty(
        "Elevator PID Slot",
        () -> {
          return currentSlotValue == 0 ? "SCORING" : "CLIMBING";
        },
        null);
    /*builder.addBooleanProperty("Elevator Limit Switch HOME ", () -> getLimitSwitchHome(), null);
    builder.addBooleanProperty(
        "Elevator Limit Switch BELOW HOME", () -> getLimitSwitchBelowHome(), null);
    builder.addBooleanProperty("Elevator Limit Switch Switch TOP", () -> getLimitSwitchTop(), null);*/

    builder.addBooleanProperty("Allow Elevator Movement", () -> allowElevatorMovement, null);
    builder.addDoubleProperty("Canrange distance INCHES", ()->Units.metersToInches(canrange.getDistance().getValueAsDouble()), null);
    builder.addDoubleProperty("Elevator voltage commanded", ()->leftMotor.getMotorVoltage().getValueAsDouble(), null);
    builder.addBooleanProperty("arm movement allowed", this::armMovementAllowed, null);
  }
}
