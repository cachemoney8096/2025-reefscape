// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoScoringPrepSequence;
import frc.robot.commands.AutoScoringSequence;
import frc.robot.commands.IntakeSequenceManual;
import frc.robot.commands.NewHomeSequence;
import frc.robot.commands.PrepScoreAndDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer extends SubsystemBase {
    /* Drivetrain config */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(
            DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(
        OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(
        OperatorConstants.OPERATOR_CONTROLLER_PORT);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private CANrange distanceSensor = new CANrange(27);

    /* Auto chooser */
    private final SendableChooser<Command> autoChooser;

    /* Drive control values for heading and vision */
    public double desiredHeadingDeg = 0.0;
    private double visionBasedX = 0.0;
    private double visionBasedY = 0.0;

    private Consumer<Double> headingSetter = (Double d) -> {
    this.desiredHeadingDeg = d;
    };

    /* RR Velocity setter */
    private double driveToIntakeXPower = 0.0;

    /* Vision offsets */
    private double visionOffsetX = 0.0;
    private double visionOffsetY = 0.0;

    /* Vision velocity setters */
    private BiConsumer<Double, Double> visionVelocitySetter = (Double x, Double y) -> {
    this.visionBasedX = x.doubleValue();
    this.visionBasedY = y.doubleValue();
    };

    private Supplier<Boolean> joystickInput = () -> {
    return Math.abs(MathUtil.applyDeadband(driverController.getRightX(), 0.05)) > 0
        || Math.abs(MathUtil.applyDeadband(driverController.getLeftX(), 0.05)) > 0
        || Math.abs(MathUtil.applyDeadband(driverController.getLeftY(), 0.05)) > 0;
    };

    /* Robot centric controller */
    private boolean isManualRobotCentric = false;

    boolean isBlue = true;

    private final double deadband = 0.05;

    /* Drive controller */
    private Supplier<SwerveRequest> driveCommandSupplier = () -> driveCommand();

    /* Subsystems */
    public Arm arm;
    public Claw claw;
    public Climb climb;
    public Elevator elevator;
    public Lights lights;

    public String pathCmd;

    /* Prep states */
    public ElevatorHeight preppedHeight = ElevatorHeight.SCORE_L2;
    public IntakeSequenceManual.Location preppedIntakeLocation = IntakeSequenceManual.Location.LEFT;
    public PrepScoreAndDrive.Location preppedScoringLocation = PrepScoreAndDrive.Location.LEFT;

    /* Vision alignment controllers and variables */
    PIDController visionXController = new PIDController(1.0, 0.0, 0.0); // input meters output -1 to 1 (percent direction)
    PIDController visionYController = new PIDController(1.0, 0.0, 0.0); // input meters output -1 to 1 (percent direction)
    
    Pose3d tagPoseRobotSpaceInstance;
    Pose3d tagPoseRobotSpaceCurrent;
    Pose2d robotPoseFieldSpace;
    Pose2d targetPoseFieldSpace;

    /* Distance alignment */
    double offsetMeters = 0.53;

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
    // Warmup PathPlanner to avoid Java pauses
    FollowPathCommand.warmupCommand().schedule();

    /* Subsystems */
    arm = new Arm();
    claw = new Claw();
    climb = new Climb();
    elevator = new Elevator();
    lights = new Lights();

    /* Auto chooser */
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);
    
    /* Field centric heading controller */
    fieldCentricFacingAngle.HeadingController.setPID(6.7, 0.0001, 0.02);

    registerNamedCommands();

    zeroRobot();

    /* Configure controller bindings */
    configureDriverBindings();
    configureOperatorBindings();
    // configureDebugBindings();

    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);

    /* Shuffleboard */
    Shuffleboard.getTab("Subsystems").add(arm.getName(), arm);
    Shuffleboard.getTab("Subsystems").add(claw.getName(), claw);
    Shuffleboard.getTab("Subsystems").add(elevator.getName(), elevator);
    Shuffleboard.getTab("Subsystems").add("RobotContainer", this);

    SmartDashboard.putData(autoChooser);
    }

    private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "AUTO SCORING SEQUENCE",
        new SequentialCommandGroup(
            new InstantCommand(() -> pathCmd = "AUTO SCORING SEQUENCE"),
            new AutoScoringSequence(claw)));

    NamedCommands.registerCommand(
        "AUTO SCORING PREP SEQUENCE", 
        new SequentialCommandGroup(
            new InstantCommand(() -> pathCmd = "AUTO SCORING PREP SEQUENCE"),
            new AutoScoringPrepSequence(elevator, arm, lights)));
    }

    private void zeroRobot() {
    drivetrain.seedFieldCentric();

    if(DriverStation.getAlliance().isPresent()){
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            this.desiredHeadingDeg = 0.0;

        }
        else{
            this.desiredHeadingDeg = 180.0;
            isBlue = false;
        }
    }
    else{
        this.desiredHeadingDeg = 0.0; // Default to blue if we are cooked 💀
    }

    drivetrain.resetPose(new Pose2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), Rotation2d.fromDegrees(isBlue?0:180)));

    drivetrain.registerTelemetry(logger::telemeterize);
    }

    private SwerveRequest driveCommand() {
    {
        double rotationJoystickInput = -MathUtil.applyDeadband(driverController.getRightX(), deadband);
        double visionX = MathUtil.applyDeadband(visionBasedX, deadband);
        double visionY = MathUtil.applyDeadband(visionBasedY, deadband);
    
        double xVelocity;
        double yVelocity;
    
        if (Math.abs(visionX) > 0.0 || Math.abs(visionY) > 0.0) {
            /* If vision is present, set velocities to vision */
            xVelocity = visionX;
            yVelocity = visionY;
        } else {
            /* Else set velocity based on left stick */
            xVelocity = -driverController.getLeftY() * MaxSpeed;
            yVelocity = -driverController.getLeftX() * MaxSpeed;
        }
    
        /* Rotational veloity based on right stick */
        double rotationVelocity = -driverController.getRightX() * MaxAngularRate;
    
        if (isManualRobotCentric) {
            /* Is robot centric */
            return robotCentric
            .withVelocityX(xVelocity) 
            .withVelocityY(yVelocity) 
            .withRotationalRate(rotationVelocity);
        } else if (Math.abs(rotationJoystickInput) > 0.0) {
            /* If rotation stick is being used */
            desiredHeadingDeg = drivetrain.getState().Pose.getRotation().getDegrees();
    
            return drive
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(rotationVelocity);
        } else {
            return fieldCentricFacingAngle
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withTargetDirection(
                    Rotation2d.fromDegrees(isBlue?desiredHeadingDeg:(desiredHeadingDeg + 180))); 
        }
    }
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureDriverBindings() {
    /* Set drivetrain control command */
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(driveCommandSupplier));
    
    Command rumbleBriefly = new SequentialCommandGroup(
        new InstantCommand(
            () -> {
                driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            }),
        new WaitCommand(0.25),
        new InstantCommand(
            () -> {
                driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            }));

    // INTAKE
    driverController
        .leftTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                new IntakeSequenceManual(arm, elevator, claw, ()->preppedIntakeLocation, headingSetter).finallyDo(()->claw.stopMotors()),
                new SequentialCommandGroup(
                    new WaitUntilCommand(()->{
                        double distanceMeters = distanceSensor.getDistance().getValueAsDouble() - offsetMeters;
                        this.driveToIntakeXPower = -0.5;
                        return distanceMeters < 0.05;
                    }).finallyDo(()->this.driveToIntakeXPower = 0.0)).until(joystickInput::get)
                )
            );

    // HOME
    driverController
        .leftBumper()
        .onTrue(new NewHomeSequence(arm, elevator, claw));
    
    // PREP SCORE
    driverController
        .rightBumper()
        .onTrue(
            new PrepScoreAndDrive(elevator, arm, ()->preppedHeight, ()->preppedScoringLocation, visionVelocitySetter, headingSetter, joystickInput, drivetrain, pathCmd, ()->desiredHeadingDeg)
        );

    // SCORE
    driverController
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                    claw.runMotorsScoring();
                }));

    driverController
        .rightTrigger()
        .onFalse(
            new InstantCommand(
                () -> {
                    claw.stopMotors();
                }));

    driverController
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                    //drivetrain.seedFieldCentric();
                    drivetrain.resetPose(new Pose2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), Rotation2d.fromDegrees(isBlue?0.0:180.0)));
                    desiredHeadingDeg = isBlue?0.0:180.0;
                }));

    // Cardinals

    driverController
        .a()
        .onTrue(new InstantCommand(() -> this.desiredHeadingDeg = isBlue ? 180.0 : 0.0));

    driverController
        .b()
        .onTrue(new InstantCommand(() -> this.desiredHeadingDeg = isBlue ? 270.0 : 90.0));

    driverController
        .x()
        .onTrue(new InstantCommand(() -> this.desiredHeadingDeg = isBlue ? 90.0 : 270.0));

    driverController
        .y()
        .onTrue(new InstantCommand(() -> this.desiredHeadingDeg = isBlue ? 0.0 : 180.0));

    driverController
        .povDown()
        .onTrue(
            new InstantCommand(()->this.desiredHeadingDeg = this.desiredHeadingDeg - LimelightHelpers.getTX(Constants.LIMELIGHT_FRONT_NAME))
        );

    driverController
        .povLeft()
        .whileTrue(
            new InstantCommand(()->{
            if(drivetrain.getState().Pose.getRotation().getDegrees() - desiredHeadingDeg < 3.0){
                this.desiredHeadingDeg = this.desiredHeadingDeg - LimelightHelpers.getTX(Constants.LIMELIGHT_FRONT_NAME);
            }
            }));

    
    }

    private void configureOperatorBindings() {
    operatorController
        .rightTrigger()
        .whileTrue(new InstantCommand(() -> claw.rollerMotor.set(0.7)));

    operatorController.rightTrigger().onFalse(new InstantCommand(() -> claw.rollerMotor.set(0.0)));

    operatorController.leftTrigger().whileTrue(new InstantCommand(() -> claw.runMotorsOuttake()));
    operatorController.leftTrigger().onFalse(new InstantCommand(() -> claw.stopMotors()));

    operatorController
        .leftBumper()
        .onTrue(new InstantCommand(() -> preppedScoringLocation = PrepScoreAndDrive.Location.LEFT));
    operatorController
        .rightBumper()
        .onTrue(
            new InstantCommand(() -> preppedScoringLocation = PrepScoreAndDrive.Location.RIGHT));

    operatorController
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                    preppedHeight = ElevatorHeight.SCORE_L3;
                }));

    operatorController
        .x()
        .onTrue(new InstantCommand(() -> preppedHeight = ElevatorHeight.SCORE_L2));

    operatorController
        .a()
        .onTrue(new InstantCommand(() -> preppedHeight = ElevatorHeight.SCORE_L1));

    operatorController.b().onTrue(new InstantCommand(() -> isManualRobotCentric = !isManualRobotCentric));

    operatorController.povUp().onTrue(new InstantCommand(()->visionOffsetX+=0.05));
    operatorController.povDown().onTrue(new InstantCommand(()->visionOffsetX-=0.05));
    operatorController.povLeft().onTrue(new InstantCommand(()->visionOffsetY+=0.05));
    operatorController.povRight().onTrue(new InstantCommand(()->visionOffsetY-=0.05));

    }

    // Adds Debug Bindings

    private void configureDebugBindings() {
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(driveCommandSupplier) // Drive counterclockwise with negative X (left)
    );
    
    // Set Elevator to score_L3
    driverController
        .rightTrigger()
        .onTrue(new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.SCORE_L3)));
    
    // Reset Elevator
    driverController
        .rightBumper()
        .onTrue(new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)));

    // Set Arm postition to L3
    driverController
        .a()
        .onTrue(new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.L3)));
    
    // Reset Arm
    driverController
        .b()
        .onTrue(new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.HOME)));
    
    // Set Arm to intake
    driverController
        .x()
        .onTrue(new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.INTAKE)));

    // Intake Claw
    driverController
        .povUp()
        .onTrue(new InstantCommand(() -> claw.runMotorsIntaking()));

    // Reset Claw
    driverController
        .povUp()
        .onFalse(new InstantCommand(() -> claw.stopMotors()));

    // Vision Testing controls
    driverController.povUp().onTrue(
        /* Vision command */
        new SequentialCommandGroup(
            new InstantCommand(()->{
                visionXController.reset();
                visionYController.reset();

                /* According to the Limelight, Y rotation is yaw */
                final Rotation3d tagRot = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME).getRotation();
                this.desiredHeadingDeg -= Math.toDegrees(tagRot.getY()); 
            }),
            new WaitUntilCommand(()->
                Math.abs(drivetrain.getState().Pose.getRotation().getDegrees() - desiredHeadingDeg) < 10.0),
            new InstantCommand(()->
                /* According to the Limelight, XZ plane is floor */                                
                tagPoseRobotSpaceInstance = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME)),
            new WaitUntilCommand(() -> {
                if (tagPoseRobotSpaceInstance.getZ() == 0.0 && tagPoseRobotSpaceInstance.getX() == 0.0) {
                    /* If no inital April Tag is seen, cancel command */
                    return true;
                }

                final Pose3d tagPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME);

                if (tagPoseRobotSpaceCurrent.getZ() != 0.0 && tagPoseRobotSpace.getY() != 0.0){
                    /* If April Tag is still in sight, update instance pose */
                    tagPoseRobotSpaceInstance = tagPoseRobotSpaceCurrent;
                }

                /* Converts from Limelight Pose3d to WPI conventional Pose2d */
                Pose2d tagPoseRobotSpaceWPIConvention = new Pose2d(
                    tagPoseRobotSpaceInstance.getZ() - this.visionOffsetX,
                    -tagPoseRobotSpaceInstance.getX() + this.visionOffsetY,
                    Rotation2d.fromDegrees(tagPoseRobotSpaceInstance.getRotation().getY()));

                /* Get fieldspace poses */
                final Pose2d robotPoseFieldSpace = drivetrain.getState().Pose;
                final Pose2d targetPoseFieldSpace = robotPoseFieldSpace
                    .plus(new Transform2d(new Pose2d(), tagPoseRobotSpaceWPIConvention));

                double xOutput = visionXController.calculate(
                    robotPoseFieldSpace.getX(), targetPoseFieldSpace.getX());
                double yOutput = visionYController.calculate(
                    robotPoseFieldSpace.getY(), targetPoseFieldSpace.getY());

                xOutput = MathUtil.clamp(xOutput, -1.5, 1.5);
                yOutput = MathUtil.clamp(yOutput, -1.5, 1.5);

                if (this.isBlue) {
                    visionVelocitySetter.accept(xOutput, yOutput);
                } else {
                    visionVelocitySetter.accept(-xOutput, -yOutput);
                }

                return (Math.abs(visionXController.getPositionError()) < 0.01 && Math.abs(visionYController.getPositionError()) < 0.01);

            })).until(
                /* Break vision if joystick input */
                ()->joystickInput.get()
            ).finallyDo(()->visionVelocitySetter.accept(0.0, 0.0))
    );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("distance offset vision X", ()->visionOffsetX, (double d)->{visionOffsetX = d;System.out.println("afdsgjsdfg");});
    builder.addDoubleProperty("distance offset vision Y", ()->visionOffsetY, (double d)->{visionOffsetY = d;});
    builder.addDoubleProperty("distance sensor offset", ()->offsetMeters, (double d)->{this.offsetMeters = d;});
    builder.addBooleanProperty("robot centric enabled", ()->isManualRobotCentric, null);
    builder.addDoubleProperty("pose heading", ()->drivetrain.getState().Pose.getRotation().getDegrees(), null);
    builder.addStringProperty("Path CMD", () -> pathCmd, null);
    builder.addDoubleProperty("odometry X", () -> drivetrain.getState().Pose.getX(), null);
    builder.addDoubleProperty("odometry Y", () -> drivetrain.getState().Pose.getY(), null);
    builder.addDoubleProperty(
        "odometry rotation deg", () -> drivetrain.getState().Pose.getRotation().getDegrees(), null);
    builder.addDoubleProperty("desired heading deg", () -> this.desiredHeadingDeg, null);
    builder.addDoubleProperty(
        "gyro rotation deg", () -> drivetrain.getPigeon2().getRotation2d().getDegrees() % 360, null);
    builder.addStringProperty(
        "Current selected auto", () -> this.getAutonomousCommand().getName(), null);
    builder.addDoubleProperty("distance range meters", ()->distanceSensor.getDistance().getValueAsDouble(), null);
    builder.addDoubleProperty("tag x", ()->LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME).getX(), null);
    builder.addDoubleProperty("tag y", ()->LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME).getY(), null);
    builder.addDoubleProperty("tag z", ()->LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME).getZ(), null);
    builder.addDoubleProperty("tag rot x", ()->LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME).getRotation().getX(), null);
    builder.addDoubleProperty("tag rot y", ()->LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME).getRotation().getY(), null);
    builder.addDoubleProperty("tag rot z", ()->LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME).getRotation().getZ(), null);
    builder.addDoubleProperty("tag calc'd y", ()->LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME).getZ()*Math.tan(LimelightHelpers.getTX(Constants.LIMELIGHT_FRONT_NAME)), null);
    
    if(DriverStation.getAlliance().isPresent()){
        builder.addBooleanProperty("is blue", ()->DriverStation.getAlliance().get() == DriverStation.Alliance.Blue, null);
    }
    }
}
