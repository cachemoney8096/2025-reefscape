package frc.robot.subsystems.claw;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase {
  public TalonFX rollerMotor = new TalonFX(RobotMap.ROLLER_MOTOR_CAN_ID, "rio");

  public DigitalInput beamBreakTop = new DigitalInput(RobotMap.CLAW_BEAM_BREAK_DIO_TOP);
  public DigitalInput beamBreakBottom = new DigitalInput(RobotMap.CLAW_BEAM_BREAK_DIO_BOTTOM);
  // public DigitalInput colorSensor = new DigitalInput(RobotMap.CLAW_COLOR_SENSOR_DIO);

  public Claw() {
    initTalons();
  }

  private void initTalons() {
    TalonFXConfigurator cfg = rollerMotor.getConfigurator();
    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    toApply.CurrentLimits.SupplyCurrentLimit = ClawCal.CLAW_ROLLERS_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimit =
        ClawCal.CLAW_ROLLERS_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimitEnable = true;
    toApply.Slot0.kP = ClawCal.CLAW_ROLLERS_P;
    toApply.Slot0.kI = ClawCal.CLAW_ROLLERS_I;
    toApply.Slot0.kD = ClawCal.CLAW_ROLLERS_D;
    toApply.Slot0.kV = ClawCal.CLAW_ROLLERS_FF;
    cfg.apply(toApply);
  }

  public boolean beamBreakTop() {
    return !beamBreakTop.get();
  }

  public boolean beamBreakBottom() {
    return !beamBreakBottom.get();
  }

  // public boolean colorSensor() {
  //   return !colorSensor.get();
  // }

  public boolean beamBreakSeesObject() {
    return beamBreakTop() || beamBreakBottom();
  }

  public void runMotorsIntaking() {
    rollerMotor.set(ClawCal.CLAW_ROLLERS_INTAKING_SPEED_PERCENT);
  }

  public void runMotorsScoring() {
    rollerMotor.set(ClawCal.CLAW_ROLLERS_SCORING_SPEED_PERCENT);
  }

  public void runMotorsOuttake() {
    rollerMotor.set(ClawCal.CLAW_ROLLERS_OUTTAKING_SPEED_PERCENT);
  }

  public void stopMotors() {
    rollerMotor.set(0.0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addBooleanProperty("Claw Beam Break Sees Object", () -> beamBreakSeesObject(), null);
    builder.addBooleanProperty("Claw Beam Break Top Status", this::beamBreakTop, null);
    builder.addBooleanProperty("Claw Beam Break Bottom Status", this::beamBreakBottom, null);
    // builder.addBooleanProperty("Claw Color Sensor Status", this::colorSensor, null);
    builder.addDoubleProperty("Claw current speed (percent)", () -> rollerMotor.get(), null);
    builder.addDoubleProperty("Output voltage commanded", ()->rollerMotor.getMotorVoltage().getValueAsDouble(), null);
    builder.addDoubleProperty("Output amps", ()->rollerMotor.getTorqueCurrent().getValueAsDouble(), null);
  }
}
