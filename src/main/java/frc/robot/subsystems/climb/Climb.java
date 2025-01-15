package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.elevator.ElevatorCal;

import java.util.Optional;
import java.util.TreeMap;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;;


public class Climb extends SubsystemBase {
    private final TalonFX climbTalonLeft = new TalonFX(RobotMap.CLIMBING_LEFT_MOTOR_CAN_ID);
    private final TalonFX climbTalonRight = new TalonFX(RobotMap.CLIMBING_RIGHT_MOTOR_CAN_ID);
    private final CANcoder climbAbsoluteEncoder = new CANcoder(RobotMap.CLIMB_ABS_ENCODER_CAN_ID);

    public enum ClimbPosition{
        PRE_CLIMB_PREP,
        CLIMBING,
        STOWED,
        CLEAR_OF_ARM,
    }

    private TreeMap<ClimbPosition, Double> climbPositionMap;
    private ClimbPosition desiredPosition = ClimbPosition.STOWED;
    private Optional<Double> lastControlledTime = Optional.empty();
    private boolean allowClimbMovement = false;

    public Climb() {
        initClimbTalons();
        climbPositionMap = new TreeMap<ClimbPosition, Double>();
        climbPositionMap.put(ClimbPosition.CLIMBING, ClimbCal.CLIMB_CLIMB_POSITION_DEGREES);
        climbPositionMap.put(ClimbPosition.STOWED, ClimbCal.CLIMB_STOWED_POSITION_DEGREES);
        climbPositionMap.put(ClimbPosition.PRE_CLIMB_PREP, ClimbCal.CLIMB_PRE_CLIMB_PREP_DEGREES);
        climbPositionMap.put(ClimbPosition.CLEAR_OF_ARM, ClimbCal.CLIMB_CLEAR_OF_ARM_DEGREES);
    }

    private void initClimbTalons() {
        TalonFXConfigurator cfgLeft = climbTalonLeft.getConfigurator();
        TalonFXConfigurator cfgRight = climbTalonRight.getConfigurator();

        TalonFXConfiguration toApply = new TalonFXConfiguration();
        toApply.CurrentLimits.SupplyCurrentLimit = ClimbCal.CLIMB_TALONS_CURRENT_LIMIT_AMPS;
        toApply.CurrentLimits.StatorCurrentLimit = ClimbCal.CLIMB_TALONS_STATOR_CURRENT_LIMIT_AMPS;
        toApply.CurrentLimits.SupplyCurrentLimitEnable = true;
        toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        toApply.Slot0.kP = ClimbCal.CLIMB_MOTOR_P;
        toApply.Slot0.kI = ClimbCal.CLIMB_MOTOR_I;
        toApply.Slot0.kD = ClimbCal.CLIMB_MOTOR_D;
        toApply.Slot0.kV = ClimbCal.CLIMB_MOTOR_FF;

        cfgLeft.apply(toApply);
        toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfgRight.apply(toApply);

        climbTalonLeft.getDutyCycle().setUpdateFrequency(ClimbCal.CLIMB_DUTY_CYCLE_UPDATE_FREQ_HZ);
        climbTalonRight.getDutyCycle().setUpdateFrequency(ClimbCal.CLIMB_DUTY_CYCLE_UPDATE_FREQ_HZ);

        Follower master = new Follower(climbTalonLeft.getDeviceID(), true);
        climbTalonRight.setControl(master);
    }

    /**
    * @return the difference between the current time and the last controlled time by the timer. If
    *     the lastControlledTime is empty (first record), then return 20 milliseconds
    */
    public double getTimeDifference() {
        if (lastControlledTime.isEmpty()) {
            lastControlledTime = Optional.of(Timer.getFPGATimestamp());
            return Constants.PERIOD_TIME_SECONDS;
        }
        final double currentTimestamp = Timer.getFPGATimestamp();
        final double timestampDifference = currentTimestamp - lastControlledTime.get();
        lastControlledTime = Optional.of(currentTimestamp);

        return timestampDifference;
    }

    public void setDesiredClimbPosition(ClimbPosition pos) {
        this.desiredPosition = pos;
        this.allowClimbMovement = true;
    }

    public void dontAllowClimbMovement() {
        this.allowClimbMovement = false;
    }
    


}
