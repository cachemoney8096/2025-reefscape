package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.MatchStateUtil;

public class DriveController {
    public final double maxAngularVelocityRadiansPerSecond = 1.5 * Math.PI; 

    public ProfiledFieldCentricFacingAngle fieldController = new ProfiledFieldCentricFacingAngle(new TrapezoidProfile.Constraints(maxAngularVelocityRadiansPerSecond, maxAngularVelocityRadiansPerSecond/0.25)).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final RobotCentric robotController = new RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public final Pigeon2 gyro;
    public final MatchStateUtil msu;
    public final CommandSwerveDrivetrain drivetrain;

    public double throttle = 1.0;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
    private double MaxAngularRate = Units.RotationsPerSecond.of(2.5).in(Units.RadiansPerSecond);

    public double desiredHeading = 0.0;

    public boolean robotCentric = false;
    public ParallelRaceGroup robotCentricDrive;

    public double fieldControllerP = 5.0;
    public double fieldControllerI = 0.0;
    public double fieldControllerD = 0.0;

    public DriveController(CommandSwerveDrivetrain drivetrain, MatchStateUtil msu, CommandXboxController driverController){
        gyro = drivetrain.getPigeon2();
        this.msu = msu;
        this.drivetrain = drivetrain;
        fieldController.HeadingController.setPID(fieldControllerP, fieldControllerI, fieldControllerD); 
        rezeroControllerAndYawToMsuDefault();
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                fieldController.withVelocityX(-driverController.getLeftY() * MaxSpeed * this.getThrottle()) 
                .withVelocityY(-driverController.getLeftX() * MaxSpeed * this.getThrottle()) 
                .withTargetDirection(Rotation2d.fromDegrees(this.getDesiredHeading())) 
            )
        );
        robotCentricDrive = new RunCommand(()->robotController.withVelocityX(-driverController.getLeftY() * MaxSpeed * this.getThrottle()) 
        .withVelocityY(-driverController.getLeftX() * MaxSpeed * this.getThrottle()) 
        .withRotationalRate(-driverController.getRightX() * MaxAngularRate * this.getThrottle()), drivetrain).until(()->!robotCentric);
    }

    public void rezeroControllerToGyro(){
        fieldController.resetProfile(gyro.getRotation2d());
        drivetrain.resetRotation(gyro.getRotation2d());
    }

    public void rezeroControllerAndYawToMsuDefault(){
        fieldController.resetProfile(Rotation2d.fromDegrees(msu.isBlue()?0:180));
        drivetrain.resetRotation(Rotation2d.fromDegrees(msu.isBlue()?0:180));
        gyro.setYaw(msu.isBlue()?0:180);
    }

    public void setRobotCentric(boolean enabled){
        robotCentric = enabled;
        if(enabled){
            robotCentricDrive.execute();
        }
    }

    public void rezeroDriveToPose(Pose2d pose){
        drivetrain.resetPose(pose);
    }

    public double getThrottle(){
        return throttle;
    }

    public void setThrottle(double t){
        throttle = t;
    }

    public double getDesiredHeading(){
        return desiredHeading;
    }

    public void setDesiredHeading(double d){
        desiredHeading = d;
    }
}
