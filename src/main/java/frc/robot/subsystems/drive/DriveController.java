package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
// import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.JoystickUtil;
import frc.robot.utils.MatchStateUtil;

public class DriveController implements Sendable {
    public final double maxAngularVelocityRadiansPerSecond = 1.5 * Math.PI; 

    public final ProfiledFieldCentricFacingAngle fieldCentricFacingAngle = new ProfiledFieldCentricFacingAngle(new TrapezoidProfile.Constraints(maxAngularVelocityRadiansPerSecond, maxAngularVelocityRadiansPerSecond/0.25)).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final ProfiledRobotCentricFacingAngle robotCentricFacingAngle = new ProfiledRobotCentricFacingAngle(new TrapezoidProfile.Constraints(maxAngularVelocityRadiansPerSecond, maxAngularVelocityRadiansPerSecond/0.25)).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // public final RobotCentric robotController = new RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public final Pigeon2 gyro;
    public final MatchStateUtil msu;
    public final CommandSwerveDrivetrain drivetrain;

    public double throttle = 1.0;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
    // private double MaxAngularRate = Units.RotationsPerSecond.of(2.5).in(Units.RadiansPerSecond);

    public double desiredHeading = 0.0;

    public double robotHeadingP = 5.0;
    public double robotHeadingI = 0.0;

    public double robotHeadingD = 0.0;

    public CommandXboxController driverController;

    public boolean robotRelativeActive = false;
    public BooleanSupplier getRobotRelativeActive = ()->robotRelativeActive;


    public boolean joysticksActive = false;

    public DriveController(CommandSwerveDrivetrain drivetrain, MatchStateUtil msu, CommandXboxController driverController){
        gyro = drivetrain.getPigeon2();
        this.msu = msu;
        this.driverController = driverController;
        this.drivetrain = drivetrain;
        fieldCentricFacingAngle.HeadingController.setPID(robotHeadingP, robotHeadingI, robotHeadingD); 
        robotCentricFacingAngle.HeadingController.setPID(robotHeadingP, robotHeadingI, robotHeadingD);
        rezeroControllerAndYawToMsuDefault();
        // rezeroControllerAndYawToMsuDefault();

        drivetrain.setDefaultCommand(
            new RunCommand(
                ()-> {
                    if (robotRelativeActive) {
                        new RunCommand(this::determineDesiredHeading).until(()->!robotRelativeActive).schedule();
                        drivetrain.applyRequest(()->robotCentricFacingAngle.withVelocityX(-driverController.getLeftY() * MaxSpeed * this.getThrottle()) 
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed * this.getThrottle()) 
                        .withTargetDirection(Rotation2d.fromDegrees(desiredHeading))).until(()->!robotRelativeActive).schedule();       
                    }
                    else {
                        new RunCommand(this::determineDesiredHeading).until(getRobotRelativeActive).schedule();
                        drivetrain.applyRequest(() ->fieldCentricFacingAngle.withVelocityX(-driverController.getLeftY() * MaxSpeed * this.getThrottle()) 
                       .withVelocityY(-driverController.getLeftX() * MaxSpeed * this.getThrottle())
                       .withTargetDirection(Rotation2d.fromDegrees(desiredHeading))).until(getRobotRelativeActive).schedule();
                    }
                }, drivetrain
            )
            /*new RunCommand(
                ()->{
                    if(robotRelativeActive){
                        new RunCommand(()->desiredHeading = drivetrain.getState().Pose.getRotation().getDegrees()).until(()->!robotRelativeActive).schedule();
                        drivetrain.applyRequest(()->robotController.withVelocityX(-driverController.getLeftY() * MaxSpeed * this.getThrottle()) 
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed * this.getThrottle()) 
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate * this.getThrottle())).until(()->!robotRelativeActive).schedule();
                        
                    }
                    else{
                        new RunCommand(this::determineDesiredHeading).until(()->robotRelativeActive).schedule();
                        drivetrain.applyRequest(() ->fieldController.withVelocityX(-driverController.getLeftY() * MaxSpeed * this.getThrottle()) 
                       .withVelocityY(-driverController.getLeftX() * MaxSpeed * this.getThrottle())
                       .withTargetDirection(Rotation2d.fromDegrees(desiredHeading))).until(getRobotRelativeActive).schedule();
                    }
                }, drivetrain
            )*/

            /*new ConditionalCommand(
                new ParallelCommandGroup(
                  new RunCommand(()->desiredHeading = drivetrain.getState().Pose.getRotation().getDegrees()),
                  drivetrain.applyRequest(()->robotController.withVelocityX(-driverController.getLeftY() * MaxSpeed * this.getThrottle()) 
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed * this.getThrottle()) 
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate * this.getThrottle()))
                ),
                new ParallelCommandGroup(
                    new RunCommand(()->determineDesiredHeading()),
                    drivetrain.applyRequest(() ->fieldController.withVelocityX(-driverController.getLeftY() * MaxSpeed * this.getThrottle()) 
                       .withVelocityY(-driverController.getLeftX() * MaxSpeed * this.getThrottle())
                       .withTargetDirection(Rotation2d.fromDegrees(desiredHeading)))
                ),
                getRobotRelativeActive
            )*/
        );
            /*new ParallelCommandGroup(
                new RunCommand(determineDesiredHeading),
                              drivetrain.runOnce(() -> fieldController.resetProfile(drivetrain.getState().Pose.getRotation()))
                              .andThen(
                                      drivetrain.applyRequest(() ->
                                          fieldController.withVelocityX(-driverController.getLeftY() * MaxSpeed * this.getThrottle()) // Drive forward with negative Y (forward)
                                              .withVelocityY(-driverController.getLeftX() * MaxSpeed * this.getThrottle()) // Drive left with negative X (left)
                                              .withTargetDirection(Rotation2d.fromDegrees(desiredHeading)) // Drive to target angle using right driverController
                                      )
                                  )
                
            )*/
          /*   drivetrain.applyRequest(()->robotController.withVelocityX(-driverController.getLeftY() * MaxSpeed * this.getThrottle()) 
        .withVelocityY(-driverController.getLeftX() * MaxSpeed * this.getThrottle()) 
        .withRotationalRate(-driverController.getRightX() * MaxAngularRate * this.getThrottle()))*/
        
        //no apply request here
        //robotCentricDrive = new RunCommand().until(()->!robotCentric);
    }

    public void determineDesiredHeading(){
        double input = JoystickUtil.squareAxis(
            MathUtil.applyDeadband(-driverController.getRightX(), 0.05))*6;
        /*if(Math.abs(input) <= 0.1 && joysticksActive){
            //desiredHeading = drivetrain.getState().Pose.getRotation().getDegrees();
            joysticksActive = false;
        }
        else if(Math.abs(input) > 0.1){
            desiredHeading += input;
            joysticksActive = true;
        }*/
        desiredHeading += input;
    }

    public void rezeroControllerToGyro(){
        fieldCentricFacingAngle.resetProfile(gyro.getRotation2d());
        robotCentricFacingAngle.resetProfile(gyro.getRotation2d());
        drivetrain.resetRotation(gyro.getRotation2d());
    }

    /** 
     * 
     */
    public void rezeroControllerAndYawToMsuDefault(){
        gyro.setYaw(msu.isBlue()?0:180);
        setDesiredHeading(msu.isBlue()?0:180);
        fieldCentricFacingAngle.resetProfile(Rotation2d.fromDegrees(msu.isBlue()?0:180));
        // robotCentricFacingAngle.resetProfile(Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()));
        // drivetrain.resetRotation(gyro.getRotation2d());
    }

    public void setRobotCentric(boolean enabled){
        if(!enabled){
            rezeroControllerToGyro();
        }
        robotRelativeActive = enabled;
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Gyro Rotation (deg)", (() -> gyro.getRotation2d().getDegrees()), null);
    }
}
