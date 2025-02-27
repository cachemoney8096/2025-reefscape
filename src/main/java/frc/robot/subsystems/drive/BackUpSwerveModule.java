package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.utils.AbsoluteEncoderChecker;
import frc.robot.utils.SparkMaxUtils;

public class BackUpSwerveModule implements Sendable {
  public final TalonFX drivingTalon;
  public final SparkMax turningSparkMax;
  private final RelativeEncoder turningRelativeEncoder;
  private final AbsoluteEncoder turningAbsoluteEncoder;
  private AbsoluteEncoderChecker turningAbsoluteEncoderChecker = new AbsoluteEncoderChecker();

  private final SparkClosedLoopController turningPIDController;

  private double chassisAngularOffsetRadians = 0;

  public TalonFXConfiguration appliedConfiguration;

  /** Desired velocity and angle. This angle includes the chassis offset. */
  public SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /** if true, set chassis speeds to half the desired value */
  public boolean throttleSpeed = false;

  /**
   * Constructs a SwerveModule and configures the driving and turning motor, encoder, and PID
   * controller.
   */
  public BackUpSwerveModule(int drivingCanId, int turningCanId, double chassisAngularOffset) {
    drivingTalon = new TalonFX(drivingCanId);
    turningSparkMax = new SparkMax(turningCanId, MotorType.kBrushless);
    chassisAngularOffsetRadians = chassisAngularOffset;

    initDriveTalon();
    SparkMaxUtils.initWithRetry(this::initTurnSpark, DriveCal.SPARK_INIT_RETRY_ATTEMPTS);

    turningRelativeEncoder = turningSparkMax.getEncoder();
    turningAbsoluteEncoder = turningSparkMax.getAbsoluteEncoder();
    turningPIDController = turningSparkMax.getClosedLoopController();

    // desiredState.angle = Rotation2d.fromRadians(turningAbsoluteEncoder.getPosition());
    desiredState.angle = Rotation2d.fromRadians(turningRelativeEncoder.getPosition());
    drivingTalon.setPosition(0);
  }

  /** Does all the initialization for the spark, return true on success */
  boolean initTurnSpark() {
    int errors = 0;

    SparkMaxConfig config = new SparkMaxConfig();

    config
        .signals
        .primaryEncoderPositionPeriodMs(DriveConstants.SPARK_MAX_ENCODER_POSITION_PERIOD_MS)
        .primaryEncoderVelocityPeriodMs(DriveConstants.SPARK_MAX_ENCODER_VELOCITY_PERIOD_MS);

    Timer.delay(0.1);

    config.absoluteEncoder.inverted(DriveConstants.TURNING_ENCODER_INVERTED);

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingMaxInput(DriveConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS)
        .positionWrappingMaxInput(DriveConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS)
        .pidf(DriveCal.TURNING_P, DriveCal.TURNING_I, DriveCal.TURNING_D, DriveCal.TURNING_FF);

    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setRadsFromGearRatio(
                turningSparkMax, DriveConstants.TURN_MODULE_RELATIVE_ENCODER_GEAR_RATIO));

    // Gear ratio 1.0 because the encoder is 1:1 with the module (doesn't involve the actual turning
    // gear ratio)
    errors +=
        SparkMaxUtils.check(
            SparkMaxUtils.UnitConversions.setRadsFromGearRatio(
                turningSparkMax, DriveConstants.TURN_MODULE_ABSOLUTE_ENCODER_GEAR_RATIO));

    config
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS);

    errors +=
        SparkMaxUtils.check(
            turningSparkMax.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    return errors == 0;
  }

  /** Does all the initialization for the spark, return true on success */
  void initDriveTalon() {
    // TODO check status codes
    TalonFXConfigurator cfg = drivingTalon.getConfigurator();
    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: check this
    toApply.Feedback.SensorToMechanismRatio =
        DriveConstants.DRIVING_MOTOR_REDUCTION / DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    toApply.CurrentLimits.SupplyCurrentLimit =
        DriveConstants.DRIVING_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.SupplyCurrentLimitEnable = true;
    toApply.CurrentLimits.StatorCurrentLimit =
        DriveConstants.DRIVING_MOTOR_STATOR_TELEOP_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimitEnable = true;
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    toApply.Slot0.kP = DriveCal.DRIVING_P;
    toApply.Slot0.kI = DriveCal.DRIVING_I;
    toApply.Slot0.kD = DriveCal.DRIVING_D;
    toApply.Slot0.kV = DriveCal.DRIVING_FF;
    appliedConfiguration = toApply;
    cfg.apply(toApply);
    final double fastUpdateFrequencyHz = 50.0; // TODO change to faster for better odometry
    final double slowUpdateFrequencyHz = 50.0;
    drivingTalon.getPosition().setUpdateFrequency(fastUpdateFrequencyHz);
    drivingTalon.getVelocity().setUpdateFrequency(fastUpdateFrequencyHz);
    drivingTalon.getClosedLoopProportionalOutput().setUpdateFrequency(slowUpdateFrequencyHz);
    drivingTalon.getClosedLoopDerivativeOutput().setUpdateFrequency(slowUpdateFrequencyHz);
    drivingTalon.getClosedLoopIntegratedOutput().setUpdateFrequency(slowUpdateFrequencyHz);
    drivingTalon.getClosedLoopFeedForward().setUpdateFrequency(slowUpdateFrequencyHz);
    drivingTalon.optimizeBusUtilization();
  }

  public void considerZeroingEncoder() {
    if (Math.abs(turningAbsoluteEncoder.getPosition()) < 0.01) {
      return;
    }
    if (Math.abs(getEncoderRelativePositionRad() - getEncoderAbsPositionRad())
        > DriveCal.TURNING_ENCODER_ZEROING_THRESHOLD_RAD) {
      turningRelativeEncoder.setPosition(getEncoderAbsPositionRad() - chassisAngularOffsetRadians);
      turningPIDController.setReference(getEncoderRelativePositionRad(), ControlType.kPosition);
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    // return new SwerveModuleState(
    //     drivingTalon.getVelocity().getValue(),
    //     new Rotation2d(turningAbsoluteEncoder.getPosition() - chassisAngularOffsetRadians));
    return new SwerveModuleState(
        drivingTalon.getVelocity().getValueAsDouble() * DriveConstants.WHEEL_DIAMETER_METERS / 2,
        new Rotation2d(turningRelativeEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    // return new SwerveModulePosition(
    //     drivingTalon.getPosition().getValue(),
    //     new Rotation2d(turningAbsoluteEncoder.getPosition() - chassisAngularOffsetRadians));
    return new SwerveModulePosition(
        drivingTalon.getPosition().getValueAsDouble() * DriveConstants.WHEEL_CIRCUMFERENCE_METERS,
        new Rotation2d(turningRelativeEncoder.getPosition()));
  }

  /** Applies slew rate. */
  public double getDesiredVelocityMps(double inputVelocityMps) {
    // Allow any decrease in desired speed
    final double prevDesiredVelocityMps = desiredState.speedMetersPerSecond;
    if (Math.abs(inputVelocityMps) < Math.abs(prevDesiredVelocityMps)) {
      return inputVelocityMps;
    }

    // If the change is less than the max accel, allow it
    final double maxAccelMpss = 15.0;
    final double loopTimeS = Constants.PERIOD_TIME_SECONDS;
    final double maxVelChangeMps = maxAccelMpss * loopTimeS;
    final double velChangeMps = inputVelocityMps - prevDesiredVelocityMps;
    if (Math.abs(velChangeMps) < maxVelChangeMps) {
      return inputVelocityMps;
    }

    // Clamp to max allowed change
    final double allowedChangeMps = MathUtil.clamp(velChangeMps, -maxVelChangeMps, maxVelChangeMps);
    return prevDesiredVelocityMps + allowedChangeMps;
  }

  /** Ensures the value a is in [0, b) */
  public static double mod(double a, double b) {
    double r = a % b;
    if (r < 0) {
      r += b;
    }
    return r;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle. Angle is relative to chassis (no offset
   *     needed).
   */
  public void setDesiredState(SwerveModuleState inputState, boolean overrideSlew) {

    // Optimize the reference state to avoid spinning further than 90 degrees.
    // inputState =
    //     SwerveModuleState.optimize(inputState, new
    // Rotation2d(turningAbsoluteEncoder.getPosition()));
    inputState.optimize(new Rotation2d(turningRelativeEncoder.getPosition()));

    // Ensure optimized state
    inputState.angle = Rotation2d.fromRadians(mod(inputState.angle.getRadians(), 2.0 * Math.PI));

    if (!overrideSlew) {
      inputState.speedMetersPerSecond = getDesiredVelocityMps(inputState.speedMetersPerSecond);
    }

    // Setting global desiredState to be optimized for the shuffleboard
    this.desiredState = inputState;

    desiredState.speedMetersPerSecond =
        this.throttleSpeed
            ? 0.8 * desiredState.speedMetersPerSecond
            : desiredState.speedMetersPerSecond;

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingTalon.setControl(
        new VelocityDutyCycle(
                this.throttleSpeed
                    ? inputState.speedMetersPerSecond * 0.8 // TODO change idk mabye
                    : inputState.speedMetersPerSecond)
            .withSlot(0));
    turningPIDController.setReference(
        inputState.angle.getRadians(), SparkMax.ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetDriveEncoder() {
    drivingTalon.setPosition(0);
  }

  public double getEncoderAbsPositionRad() {
    return turningAbsoluteEncoder.getPosition();
  }

  public double getEncoderRelativePositionRad() {
    return turningRelativeEncoder.getPosition();
  }

  public void periodic() {
    turningAbsoluteEncoderChecker.addReading(turningAbsoluteEncoder.getPosition());
  }

  public void throttleSpeed(boolean throttleSpeed) {
    this.throttleSpeed = throttleSpeed;
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Driving kP",
        () -> {
          return DriveCal.DRIVING_P;
        },
        null);
    builder.addDoubleProperty(
        "Driving kI",
        () -> {
          return DriveCal.DRIVING_I;
        },
        null);
    builder.addDoubleProperty(
        "Driving kD",
        () -> {
          return DriveCal.DRIVING_D;
        },
        null);
    builder.addDoubleProperty(
        "Driving kP output contribution",
        () -> {
          return drivingTalon.getClosedLoopProportionalOutput().getValue();
        },
        null); // TODO find a setter? i couldn't
    builder.addDoubleProperty(
        "Driving kI output contribution",
        () -> {
          return drivingTalon.getClosedLoopIntegratedOutput().getValue();
        },
        null);
    builder.addDoubleProperty(
        "Driving kD output contribution",
        () -> {
          return drivingTalon.getClosedLoopDerivativeOutput().getValue();
        },
        null);
    builder.addDoubleProperty(
        "Driving kFF",
        () -> {
          return drivingTalon.getClosedLoopFeedForward().getValue();
        },
        null);
    builder.addDoubleProperty("Turning kP", turningSparkMax.configAccessor.closedLoop::getP, null);
    builder.addDoubleProperty("Turning kI", turningSparkMax.configAccessor.closedLoop::getI, null);
    builder.addDoubleProperty("Turning kD", turningSparkMax.configAccessor.closedLoop::getD, null);
    builder.addDoubleProperty(
        "Turning kFF", turningSparkMax.configAccessor.closedLoop::getFF, null);
    builder.addDoubleProperty(
        "Driving Vel (m/s)",
        () -> {
          return drivingTalon.getVelocity().getValueAsDouble()
              * DriveConstants.WHEEL_DIAMETER_METERS
              / 2;
        },
        null);
    builder.addDoubleProperty(
        "Steering Pos (rad) - absolute", turningAbsoluteEncoder::getPosition, null);
    builder.addDoubleProperty(
        "Steering Pos (rad) - relative", turningRelativeEncoder::getPosition, null);
    builder.addDoubleProperty(
        "Desired Vel (m/s)",
        () -> {
          return desiredState.speedMetersPerSecond;
        },
        null);
    builder.addDoubleProperty(
        "Desired Steer (rad)",
        () -> {
          return desiredState.angle.getRadians();
        },
        null);
    builder.addBooleanProperty(
        "Turning encoder connected", turningAbsoluteEncoderChecker::encoderConnected, null);
  }
}
