package frc.robot.subsystems.claw;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase {
    public TalonFX rollerMotor = new TalonFX(ClawConstants.ROLLER_MOTOR_CAN_ID);

    public DigitalInput beamBreak1 = new DigitalInput(ClawConstants.BEAM_BREAK_DIO_1);
    public DigitalInput beamBreak2 = new DigitalInput(ClawConstants.BEAM_BREAK_DIO_2);
    public DigitalInput beamBreak3 = new DigitalInput(ClawConstants.BEAM_BREAK_DIO_3);


    public Claw(){
        initTalons();
    }

    private void initTalons(){
        TalonFXConfigurator cfg = rollerMotor.getConfigurator();
        TalonFXConfiguration toApply = new TalonFXConfiguration();
        toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO change this to make run intake methods work
        toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast; //TODO change this based on how tight the compression is
        toApply.CurrentLimits.SupplyCurrentLimit = ClawCal.CLAW_ROLLERS_SUPPLY_CURRENT_LIMIT_AMPS;
        toApply.CurrentLimits.StatorCurrentLimit = ClawCal.CLAW_ROLLERS_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        toApply.CurrentLimits.StatorCurrentLimitEnable = true;
        toApply.Slot0.kP = ClawCal.CLAW_ROLLERS_P;
        toApply.Slot0.kI = ClawCal.CLAW_ROLLERS_I;
        toApply.Slot0.kD = ClawCal.CLAW_ROLLERS_D;
        toApply.Slot0.kV = ClawCal.CLAW_ROLLERS_FF;
        cfg.apply(toApply);
    }

    public boolean beamBreak1(){
        return !beamBreak1.get();
    }
    
    public boolean beamBreak2(){
        return !beamBreak2.get();
    }

    public boolean beamBreak3(){
        return !beamBreak3.get();
    }

    public boolean beamBreakSeesObject(){
        return beamBreak1() || beamBreak2();
    }
    public boolean beamBreakNotSeesObject(){
        return !(beamBreak1() || beamBreak2());
    }


    public void runMotorsIntaking(){
        rollerMotor.set(ClawCal.CLAW_ROLLERS_INTAKING_SPEED_PERCENT);
    }

    public void runMotorsScoring(){
        rollerMotor.set(ClawCal.CLAW_ROLLERS_SCORING_SPEED_PERCENT);
    }

    public void stopMotors(){
        rollerMotor.set(0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty(
            "Beam Break 1 Status", this::beamBreak1, null);
        builder.addBooleanProperty(
            "Beam Break 2 Status", this::beamBreak2, null);
    }
}
