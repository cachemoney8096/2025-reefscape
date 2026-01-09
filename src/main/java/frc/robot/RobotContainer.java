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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    /* Instance Variables */

    /* Controllers */
    private final CommandXboxController driverController = new CommandXboxController(
        OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(
        OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private final double joystickDeadband = 0.05;

    private Supplier<Boolean> rotationalJoystickInput = () -> {
        return Math.abs(MathUtil.applyDeadband(driverController.getRightX(), joystickDeadband)) > 0.0;
        };

    private Supplier<Boolean> positionalJoystickInput = () -> {
        return Math.abs(MathUtil.applyDeadband(driverController.getLeftX(), joystickDeadband)) > 0.0
            || Math.abs(MathUtil.applyDeadband(driverController.getLeftY(), joystickDeadband)) > 0.0;
        };

    private Supplier<Boolean> joystickInput = () -> {
        return rotationalJoystickInput.get() || positionalJoystickInput.get();
        };

    /* Auto chooser */
    private final SendableChooser<Command> autoChooser;

    /* Distance Sensor */
    private CANrange distanceSensor = new CANrange(27);
    private double distanceOffsetMeters = 0.53;

    /* Drive control */
    private Supplier<SwerveRequest> driveController = this::driveCommand;
    public double desiredHeadingDeg = 0.0;
    private double visionBasedX = 0.0;
    private double visionBasedY = 0.0;

    private Consumer<Double> headingSetter = (Double d) -> {
        this.desiredHeadingDeg = d;
    };

    private BiConsumer<Double, Double> visionVelocitySetter = (Double x, Double y) -> {
        this.visionBasedX = x.doubleValue();
        this.visionBasedY = y.doubleValue();
    };

    /* Drivetrain config */
    private final double driveDeadband = 0.1;

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * driveDeadband)
        .withRotationalDeadband(MaxAngularRate * driveDeadband) 
        .withDriveRequestType(
            DriveRequestType.OpenLoopVoltage); /* Use open-loop control for drive motors */

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * driveDeadband)
        .withRotationalDeadband(MaxAngularRate * driveDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * driveDeadband)
        .withRotationalDeadband(MaxAngularRate * driveDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Vision variables */
    private double visionOffsetX = 0.0;
    private double visionOffsetY = 0.0;

    private PIDController visionXController = new PIDController(1.0, 0.0, 0.0); // input meters output -1 to 1 (percent direction)
    private PIDController visionYController = new PIDController(1.0, 0.0, 0.0); // input meters output -1 to 1 (percent direction)    
    private Pose3d tagPoseRobotSpaceInstance;
    private Pose3d tagPoseRobotSpaceCurrent;    

    /* Robot centric controller */
    private boolean isManualRobotCentric = false;

    /* Team color */
    public boolean isBlue = true;

    /* Subsystems */
    public Arm arm;
    public Claw claw;
    public Climb climb;
    public Elevator elevator;
    public Lights lights;

    public String autoPathCmd;

    /* Prep states */
    public ElevatorHeight preppedHeight = ElevatorHeight.SCORE_L2;
    public IntakeSequenceManual.Location preppedIntakeLocation = IntakeSequenceManual.Location.LEFT;
    public PrepScoreAndDrive.Location preppedScoringLocation = PrepScoreAndDrive.Location.LEFT;

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        /* Named commands must be registed first immediately */
        registerNamedCommands();

        /* Warmup PathPlanner to avoid Java pauses */
        FollowPathCommand.warmupCommand().schedule();

        /* Init subsystems */
        arm = new Arm();
        claw = new Claw();
        climb = new Climb();
        elevator = new Elevator();
        lights = new Lights();

        /* Auto chooser */
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        /* Field centric heading controller */
        fieldCentricFacingAngle.HeadingController.setPID(6.7, 0.0001, 0.02); /* 6.7 💀 */

        isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue; /* Default to blue if we are cooked 💀 */

        zeroRobot();

        /* Configure controller bindings */
        configureDriverBindings();
        configureOperatorBindings();
        
        if (false) configureDebugBindings(); /* Change to true for debug bindings */

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
                new InstantCommand(() -> autoPathCmd = "AUTO SCORING SEQUENCE"),
                new AutoScoringSequence(claw)));

        NamedCommands.registerCommand(
            "AUTO SCORING PREP SEQUENCE", 
            new SequentialCommandGroup(
                new InstantCommand(() -> autoPathCmd = "AUTO SCORING PREP SEQUENCE"),
                new AutoScoringPrepSequence(elevator, arm, lights)));
    }

    private void zeroRobot() {
        drivetrain.seedFieldCentric();

        this.desiredHeadingDeg = isBlue ? 0.0 : 180.0 ;

        drivetrain.resetPose(new Pose2d(
            drivetrain.getState().Pose.getX(), 
            drivetrain.getState().Pose.getY(), 
            Rotation2d.fromDegrees(isBlue ? 0.0: 180)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private SwerveRequest driveCommand() {
        double visionX = MathUtil.applyDeadband(visionBasedX, joystickDeadband);
        double visionY = MathUtil.applyDeadband(visionBasedY, joystickDeadband);
    
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
        } else if (rotationalJoystickInput.get()) {
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
            drivetrain.applyRequest(driveController));
        
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

        /* Intake */
        driverController
            .leftTrigger()
            .whileTrue(
                new ParallelCommandGroup(
                    new IntakeSequenceManual(arm, elevator, claw, ()->preppedIntakeLocation, headingSetter).finallyDo(()->claw.stopMotors()),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(()->{
                            double distanceMeters = distanceSensor.getDistance().getValueAsDouble() - distanceOffsetMeters;
                            return distanceMeters < 0.05;
                        })).until(joystickInput::get)
                    )
                );

        /* Home */
        driverController
            .leftBumper()
            .onTrue(new NewHomeSequence(arm, elevator, claw));
        
        /* Prep Score */
        driverController
            .rightBumper()
            .onTrue(
                new PrepScoreAndDrive(elevator, arm, ()->preppedHeight, ()->preppedScoringLocation, visionVelocitySetter, headingSetter, joystickInput, drivetrain, autoPathCmd, ()->desiredHeadingDeg)
            );

        /* Score */
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
                        drivetrain.resetPose(new Pose2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), Rotation2d.fromDegrees(isBlue?0.0:180.0)));
                        desiredHeadingDeg = isBlue ? 0.0 : 180.0;
                    }));

        /* Cardinals */ 
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

        /* Outtake */
        operatorController.leftTrigger().whileTrue(new InstantCommand(() -> claw.runMotorsOuttake()));
        operatorController.leftTrigger().onFalse(new InstantCommand(() -> claw.stopMotors()));

        /* Change scoring location */
        operatorController
            .leftBumper()
            .onTrue(new InstantCommand(() -> preppedScoringLocation = PrepScoreAndDrive.Location.LEFT));
        
        operatorController
            .rightBumper()
            .onTrue(new InstantCommand(() -> preppedScoringLocation = PrepScoreAndDrive.Location.RIGHT));

        /* Change scoring height */
        operatorController
            .y()
            .onTrue(new InstantCommand(() -> preppedHeight = ElevatorHeight.SCORE_L3));

        operatorController
            .x()
            .onTrue(new InstantCommand(() -> preppedHeight = ElevatorHeight.SCORE_L2));

        operatorController
            .a()
            .onTrue(new InstantCommand(() -> preppedHeight = ElevatorHeight.SCORE_L1));

        /* Toggle robot centric */
        operatorController.b().onTrue(new InstantCommand(() -> isManualRobotCentric = !isManualRobotCentric));
    }

    private void configureDebugBindings() {
        /* Set elevator to score_L3 */ 
        driverController
            .rightTrigger()
            .onTrue(new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.SCORE_L3)));
        
        /* Reset elevator */ 
        driverController
            .rightBumper()
            .onTrue(new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.HOME)));

        /* Set arm to L3 */ 
        driverController
            .a()
            .onTrue(new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.L3)));
        
        /* Resets arm */ 
        driverController
            .b()
            .onTrue(new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.HOME)));
        
        /* Sets arm to intake */ 
        driverController
            .x()
            .onTrue(new InstantCommand(() -> arm.setDesiredPosition(ArmPosition.INTAKE)));

        /* Intake with claw */ 
        driverController
            .povUp()
            .onTrue(new InstantCommand(() -> claw.runMotorsIntaking()));

        driverController
            .povUp()
            .onFalse(new InstantCommand(() -> claw.stopMotors()));
        
        /* Adjust vision offsets */
        operatorController.povUp().onTrue(new InstantCommand(()->visionOffsetX+=0.05));
        operatorController.povDown().onTrue(new InstantCommand(()->visionOffsetX-=0.05));
        operatorController.povLeft().onTrue(new InstantCommand(()->visionOffsetY+=0.05));
        operatorController.povRight().onTrue(new InstantCommand(()->visionOffsetY-=0.05));

        /* Tests vision */
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
        builder.addDoubleProperty("distance sensor offset", ()->distanceOffsetMeters, (double d)->{this.distanceOffsetMeters = d;});
        builder.addBooleanProperty("robot centric enabled", ()->isManualRobotCentric, null);
        builder.addDoubleProperty("pose heading", ()->drivetrain.getState().Pose.getRotation().getDegrees(), null);
        builder.addStringProperty("path CMD", () -> autoPathCmd, null);
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
        builder.addBooleanProperty("is blue", ()->isBlue, null);
    }
}