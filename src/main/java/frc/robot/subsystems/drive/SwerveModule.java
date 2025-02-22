package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.robot.utils.AbsoluteEncoderChecker;
import frc.robot.utils.SparkMaxUtils;

public class SwerveModule implements Sendable {
  public final TalonFX drivingTalon;
  public final SparkMax turningSparkMax;
  private final RelativeEncoder turningRelativeEncoder;

  ;

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
  public SwerveModule(int drivingCanId, int turningCanId, double chassisAngularOffset) {
    drivingTalon = new TalonFX(drivingCanId);
    turningSparkMax = new SparkMax(turningCanId, MotorType.kBrushless);
    chassisAngularOffsetRadians = chassisAngularOffset;

    initDriveTalon();
    SparkMaxUtils.initWithRetry(this::initTurnSpark, DriveCal.SPARK_INIT_RETRY_ATTEMPTS);

    turningRelativeEncoder = turningSparkMax.getEncoder();
    turningPIDController = turningSparkMax.getClosedLoopController();

    // desiredState.angle = Rotation2d.fromRadians(turningAbsoluteEncoder.getPosition());
    desiredState.angle = Rotation2d.fromRadians(turningRelativeEncoder.getPosition());
    drivingTalon.setPosition(0);
  }

  /** Does all the initialization for the spark, return true on success */
  boolean initTurnSpark() {
    int errors = 0;
    return errors == 0;
  }

  /** Does all the initialization for the spark, return true on success */
  void initDriveTalon() {
    
  }

  public void considerZeroingEncoder() {
    
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
    return new SwerveModuleState();
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
    return new SwerveModulePosition();
  }

  /** Applies slew rate. */
  public double getDesiredVelocityMps(double inputVelocityMps) {
   
    return  0.0;
  }

  /** Ensures the value a is in [0, b) */
  public static double mod(double a, double b) {
    double r = a % b;
    return r;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle. Angle is relative to chassis (no offset
   *     needed).
   */
  public void setDesiredState(SwerveModuleState inputState, boolean overrideSlew) {
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetDriveEncoder() {
  }

  public double getEncoderAbsPositionRad() {
    return 0.0;
  }

  public double getEncoderRelativePositionRad() {
    return 0.0;
  }

  public void periodic() {
   
  }

  public void throttleSpeed(boolean throttleSpeed) {
  
  }

  public void initSendable(SendableBuilder builder) {
    
  }
}