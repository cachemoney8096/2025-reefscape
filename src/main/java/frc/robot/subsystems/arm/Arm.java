package frc.robot.subsystems.arm;

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
import frc.robot.Constants;
import frc.robot.RobotMap;
import java.util.TreeMap;

public class Arm extends SubsystemBase {
  private final TalonFX armMotorLeft = new TalonFX(RobotMap.LEFT_ARM_MOTOR_CAN_ID, "rio");
  private final TalonFX armMotorRight = new TalonFX(RobotMap.RIGHT_ARM_MOTOR_CAN_ID, "rio");

  private final CANcoder armLeftEncoderAbs = new CANcoder(RobotMap.ARM_ABS_ENCODER_CAN_ID, "rio");
  // private final Encoder armLeftEncoderAbs =
  // new Encoder(RobotMap.ARM_ABS_ENCODER_DIO_A, RobotMap.ARM_ABS_ENCODER_DIO_B);

  // trapezoidal motion profiling to account for large jumps in velocity which result in large error
  private final TrapezoidProfile trapezoidProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ArmCal.ARM_MOTOR_MAX_VELOCITY_RPS, ArmCal.ARM_MOTOR_MAX_ACCERLATION_RPS_SQUARED));
  private TrapezoidProfile.State tSetpoint = new TrapezoidProfile.State();

  public enum ArmPosition {
    HOME,
    INTAKE,
    DEEP_CLIMB,
    L1,
    L2,
    L3,
    L4
  }

  /** Map each of our arm positions to an actual position on our arm (degrees) */
  public final TreeMap<ArmPosition, Double> armPositions = new TreeMap<ArmPosition, Double>();

  private ArmPosition armDesiredPosition = ArmPosition.HOME;

  public Arm() {

    armPositions.put(ArmPosition.HOME, ArmCal.ARM_POSITION_HOME_DEGREES);
    armPositions.put(ArmPosition.INTAKE, ArmCal.ARM_POSITION_INTAKE_DEGREES);
    armPositions.put(ArmPosition.DEEP_CLIMB, ArmCal.ARM_POSITION_DEEP_CLIMB_DEGREES);
    armPositions.put(ArmPosition.L1, ArmCal.ARM_POSITION_L1_DEGREES);
    armPositions.put(ArmPosition.L2, ArmCal.ARM_POSITION_L2_DEGREES);
    armPositions.put(ArmPosition.L3, ArmCal.ARM_POSITION_L3_DEGREES);
    armPositions.put(ArmPosition.L4, ArmCal.ARM_POSITION_L4_DEGREES);
    initArmTalons();
    rezeroArm();
    // armLeftEncoderAbs.setDistancePerPulse(Constants.DEGREES_PER_REV_THROUGH_BORE_ABS_ENCODER_PULSE);
  }

  private void initArmTalons() {
    TalonFXConfiguration toApply = new TalonFXConfiguration();

    toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
    this.armDesiredPosition = armPosition;
  }

  public double getPositionArmRotationsReal(){
    // if(armLeftEncoderAbs.getAbsolutePosition().getValueAsDouble() >= 0){
    //   return armLeftEncoderAbs.getAbsolutePosition().getValueAsDouble();
    // }
    // else{
    //   return 1.0 + armLeftEncoderAbs.getAbsolutePosition().getValueAsDouble();
    // }
    return 0.5 + armLeftEncoderAbs.getAbsolutePosition().getValueAsDouble();
  }

  public void rezeroArm() { //this is stupid
    armMotorLeft.setPosition(getPositionArmRotationsReal()*90);
    // armMotorLeft.setPosition(armLeftEncoderAbs.getDistance());
   // armMotorLeft.setPosition(getPositionArmRotationsReal()*90);
    tSetpoint =
        new TrapezoidProfile.State(getPositionArmRotationsReal() * 90, 0.0);
    // new TrapezoidProfile.State(armLeftEncoderAbs.getDistance(), 0.0);
  }

  // Account for PID when setting position of our arm
  public void controlPosition(double inputPositionDegrees) {
    // System.out.println("control pos\n");
    // goal position (rotations) w/ velocity at position (0?)
    /*PositionVoltage tRequest = new PositionVoltage(0.0).withSlot(0);
    TrapezoidProfile.State tGoal = new TrapezoidProfile.State(inputPositionDegrees / 360.0, 0);
    // set next setpoint, where t = periodic interval (20ms)
    
    tSetpoint = trapezoidProfile.calculate(Constants.PERIOD_TIME_SECONDS, tSetpoint, tGoal);

    tRequest.Position = tSetpoint.position;
    tRequest.Velocity = tSetpoint.velocity;

    armMotorLeft.setControl(tRequest);*/
    final TrapezoidProfile trapezoidProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              ArmCal.ARM_MOTOR_MAX_VELOCITY_RPS, ArmCal.ARM_MOTOR_MAX_ACCERLATION_RPS_SQUARED));
    TrapezoidProfile.State tGoal = new TrapezoidProfile.State(inputPositionDegrees / 360.0 * 90, 0.0);

    System.out.println("\ngoal.pos " + tGoal.position + "\n");
    System.out.println("goal.vel " + tGoal.velocity + "\n");
    TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    // TrapezoidProfile.State setpoint =
    //     new TrapezoidProfile.State(getPositionArmRotationsReal() * 90, 0.0);

    System.out.println("\nsetpoint.pos " + setpoint.position + "\n");
    System.out.println("setpoint.vel " + setpoint.velocity + "\n");
    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    setpoint = trapezoidProfile.calculate(0.020, setpoint, tGoal);
    request.Position = setpoint.position;
    request.Velocity = setpoint.velocity;
    System.out.println("\nrequest.pos " + request.Position + "\n");
    System.out.println("request.vel " + request.Velocity + "\n");
    armMotorLeft.setControl(request);
  }

  public boolean atArmPosition(ArmPosition pos) {
    double checkPositionDeg = armPositions.get(pos);
    // double currentPositionDeg = getPositionArmRotationsReal() * 360.0;
    double currentPositionDeg = getPositionArmRotationsReal() * 360.0;

    return Math.abs(checkPositionDeg - currentPositionDeg) <= ArmCal.ARM_MARGIN_DEGREES;
  }

  public boolean atDesiredArmPosition() {
    return atArmPosition(armDesiredPosition);
  }

  public void stopArmMovement() {
    // left motor follows right motor, so armMotorRight is not necessary here
    armMotorLeft.setVoltage(0.0);
  }

  public void testArmMovementUp() {
    armMotorLeft.setVoltage(ArmCal.TEST_ARM_MOVEMENT_VOLTAGE);
  }

  public void testArmMovementDown() {
    armMotorLeft.setVoltage(-1 * ArmCal.TEST_ARM_MOVEMENT_VOLTAGE);
  }

  @Override
  public void periodic() {
    controlPosition(armPositions.get(this.armDesiredPosition)); // TODO
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addStringProperty("Arm DESIRED Pos", () -> armDesiredPosition.toString(), null);
    builder.addDoubleProperty(
        "Arm DESIRED Pos (deg)", (() -> armPositions.get(armDesiredPosition)), null);
    builder.addBooleanProperty("Arm at desired", (() -> atDesiredArmPosition()), null);

    builder.addDoubleProperty(
        "Arm Left Motor RELATIVE (deg)",
        (() -> armMotorLeft.getPosition().getValueAsDouble() * 360.0),
        null);
    builder.addDoubleProperty(
        "Arm Right Motor RELATIVE (deg)",
        (() -> armMotorRight.getPosition().getValueAsDouble() * 360.0),
        null);
    builder.addDoubleProperty(
        "Arm Left Motor ABS (deg)",
        (() -> armLeftEncoderAbs.getAbsolutePosition().getValueAsDouble() * 360.0),
        // (() -> armLeftEncoderAbs.getDistance() * 360.0),
        null);

        builder.addDoubleProperty(
          "Arm Left Motor ABS REAL (deg)",
          (() -> getPositionArmRotationsReal() * 360.0),
          // (() -> armLeftEncoderAbs.getDistance() * 360.0),
          null);

    builder.addDoubleProperty("Arm Trapezoid Setpoint Pos (revs)", () -> tSetpoint.position, null);
    builder.addDoubleProperty(
        "Arm Trapezoid Setpoint Velocity (revs/sec)", () -> tSetpoint.velocity, null);

    builder.addDoubleProperty("rotations measured", ()-> armLeftEncoderAbs.getAbsolutePosition().getValueAsDouble(), null);
    builder.addDoubleProperty("rotations 'real'", (() -> getPositionArmRotationsReal()), null);
    builder.addDoubleProperty("Output voltage commanded", ()->armMotorLeft.getMotorVoltage().getValueAsDouble(), null);
  }
}
