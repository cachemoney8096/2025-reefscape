// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.AutoIntakeSequence;
import frc.robot.commands.AutoScoringPrepSequence;
import frc.robot.commands.DeepClimbScoringSequence;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.FinishScore;
import frc.robot.commands.GoHomeSequence;
import frc.robot.commands.GoHomeSequenceFake;
import frc.robot.commands.IntakeSequenceManual;
import frc.robot.commands.NewHomeSequence;
import frc.robot.commands.PrepScoreAndDrive;
import frc.robot.commands.PrepScoreManual;
import frc.robot.commands.IntakeSequenceManual.Location;
import frc.robot.commands.autos.Push;
import frc.robot.commands.autos.S1.P2_S1_I_J;
import frc.robot.commands.autos.S1.P2_S1_J_I;
import frc.robot.commands.autos.S1.P3_S1_I_J_K;
import frc.robot.commands.autos.S1.P3_S1_J_I_K;
import frc.robot.commands.autos.S1.P4_S1_I_J_K_L;
import frc.robot.commands.autos.S1.P4_S1_J_I_K_L;
import frc.robot.commands.autos.S2.P1_S2_G;
import frc.robot.commands.autos.S2.P1_S2_H;
import frc.robot.commands.autos.S2.P2_S2_G_H;
import frc.robot.commands.autos.S2.P2_S2_H_G;
import frc.robot.commands.autos.S2.P3_S2_G_H_B;
import frc.robot.commands.autos.S2.P3_S2_H_G_A;
import frc.robot.commands.autos.S3.P2_S3_F_E;
import frc.robot.commands.autos.S3.P3_S3_E_F_D;
import frc.robot.commands.autos.S3.P3_S3_F_E_D;
import frc.robot.commands.autos.S3.P4_S3_E_F_D_C;
import frc.robot.commands.autos.S3.P4_S3_F_E_D_C;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.utils.MatchStateUtil;
import frc.robot.utils.PrepStateUtil;
import frc.robot.utils.PrepStateUtil.INTAKE_CLIMB_LOCATION;
import frc.robot.utils.PrepStateUtil.SCORE_HEIGHT;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.time.Instant;
import java.util.TreeMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable {
  private MatchStateUtil matchState;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController driverController =
    new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  /* Drive control code, this setup is genuinely the dumbest thing ever */
  private double desiredHeadingDeg = 0.0; //desired heading
  private double visionBasedX = 0.0;
  private double visionBasedY = 0.0;
  private double visionBasedRotation = 0.0;
  /* get the drivecommand to use based on control input.
  * if we are providing rotational input, use that
  * otherwise, use our heading controller, which allows for cardinals
  * rotational control input will always override and terminate a cardinal input
  */
  private double degInRange(double deg){
    return deg % 360;
  }

  private Consumer<Pair<Double, Double>> velocitySetter = (Pair<Double, Double> v)->{
        this.visionBasedX = v.getFirst().doubleValue();
        this.visionBasedY = v.getSecond().doubleValue();
    };

    private Consumer<Double> headingSetter = (Double d)->{
        this.desiredHeadingDeg = d;
    };

    private Supplier<Boolean> joystickInput = ()->{
        return Math.abs(MathUtil.applyDeadband(driverController.getRightX(), 0.05)) > 0 || Math.abs(MathUtil.applyDeadband(driverController.getLeftX(), 0.05)) > 0 || Math.abs(MathUtil.applyDeadband(driverController.getLeftY(), 0.05)) > 0;
    };

  private Supplier<SwerveRequest> driveCommand = ()->{
    double rotationJoystickInput = -MathUtil.applyDeadband(driverController.getRightX(), 0.05);
    double vx = MathUtil.applyDeadband(visionBasedX, 0.05);
    double vy = MathUtil.applyDeadband(visionBasedY, 0.05);
    if(Math.abs(rotationJoystickInput) > 0){
        desiredHeadingDeg = drivetrain.getState().Pose.getRotation().getDegrees(); // keep the desired heading updated
        return drive
        .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-driverController.getRightX() * MaxAngularRate);
    }
    else if(vx > 0 || vy > 0 || visionBasedRotation > 0){
        desiredHeadingDeg = drivetrain.getState().Pose.getRotation().getDegrees();
        return fieldCentricFacingAngle
        .withVelocityX(vx)
        .withVelocityY(vy)
        .withTargetDirection(Rotation2d.fromDegrees(desiredHeadingDeg)); // use the desired heading, in this case controlled by vision
    }
    else{
        return fieldCentricFacingAngle
        .withVelocityX(-driverController.getLeftY() * MaxSpeed)
        .withVelocityY(-driverController.getLeftX() * MaxSpeed)
        .withTargetDirection(Rotation2d.fromDegrees(desiredHeadingDeg)); // use the desired heading, either as a keep heading or for cardinals
    }
  };

  /* Subsystems */
  public Arm arm;
  public Claw claw;
  public Climb climb;
  // public DriveSubsystem drive;
  public Elevator elevator;
  public Lights lights;

  public String pathCmd = "";

  public enum PrepState {
    OFF,
    SCORE,
    CLIMB,
    INTAKE
  }

  /* Prep states */


  public PrepState prepState = PrepState.OFF;
  public IntakeSequenceManual.Location preppedIntakeLocation = IntakeSequenceManual.Location.LEFT;
  public ElevatorHeight preppedHeight = ElevatorHeight.SCORE_L3;
  public PrepScoreAndDrive.Location preppedScoringLocation = PrepScoreAndDrive.Location.LEFT;

  private PrepStateUtil prepStateUtil = new PrepStateUtil();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(MatchStateUtil ms) {
    /* Named commands here */

    /*SequentialCommandGroup intake = new SequentialCommandGroup(
        new InstantCommand(() -> pathCmd = "AUTO INTAKE SEQUENCE"),
        new AutoIntakeSequence(elevator, arm, claw, lights));

    NamedCommands.registerCommand(
        "AUTO INTAKE SEQUENCE",
        intake
        );

    NamedCommands.registerCommand("FINISH INTAKE", 
    new SequentialCommandGroup(
        new InstantCommand(() -> claw.runMotorsIntaking()),
        new WaitUntilCommand(claw::beamBreakSeesObject),
        new InstantCommand(() -> claw.stopMotors()),
        new InstantCommand(() -> lights.setLEDColor(LightCode.HAS_CORAL))
    ));

    SequentialCommandGroup prep = new SequentialCommandGroup(
        new InstantCommand(() -> pathCmd = "AUTO SCORING PREP SEQUENCE"),
        new AutoScoringPrepSequence(elevator, arm, lights));

    NamedCommands.registerCommand(
        "AUTO SCORING PREP SEQUENCE",
        prep
        );
    */
    /* auto chooser has some nice builtin functionality */
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);
    fieldCentricFacingAngle.HeadingController.setPID(2.5, 0.0, 0.0);

    /* zero everything */
    drivetrain.seedFieldCentric();
    this.desiredHeadingDeg = 0.0;

    // Warmup PathPlanner to avoid Java pauses
    FollowPathCommand.warmupCommand().schedule();

    /* Subsystems */
    arm = new Arm();
    claw = new Claw();
    climb = new Climb();
    elevator = new Elevator();
    lights = new Lights();
    

    /*NamedCommands.registerCommand(
    "AUTO SCORING SEQUENCE",
    new SequentialCommandGroup(
            new InstantCommand(() -> pathCmd = "AUTO SCORING SEQUENCE"),
                    new AutoScoringSequence(claw)
    ));*/

    SequentialCommandGroup score =
        new SequentialCommandGroup(
            new InstantCommand(()->pathCmd = "AUTO SCORING SEQUENCE"),
            new InstantCommand(() -> claw.runMotorsScoring()),
            new WaitCommand(1.0),
            new InstantCommand(() -> claw.stopMotors()));

    NamedCommands.registerCommand(
        "AUTO SCORING SEQUENCE", score);
    matchState = ms;

    // scoringLimelight = TODO: VErify removal
    //     new ScoringLimelight(
    //         ScoringLimelightConstants.SCORING_LIMELIGHT_PITCH_DEGREES,
    //         ScoringLimelightConstants.SCORING_LIMELIGHT_HEIGHT_METERS,
    //         0.0);
    // intakeLimelight =
    //     new IntakeLimelight(
    //         IntakeLimelightConstants.INTAKE_LIMELIGHT_PITCH_DEGREES,
    //         IntakeLimelightConstants.INTAKE_LIMELIGHT_HEIGHT_METERS,
    //         0.0); // ""

    drivetrain.registerTelemetry(logger::telemeterize);

    /* Configure controller bindings */
    configureDriverBindings();
    //configureOperatorBindings();

    /* Debug Bindings */
    //configureDebugBindings();

    /* Shuffleboard */
    // Shuffleboard.getTab("Subsystems").add(drivetrain.getName(), drive);
    Shuffleboard.getTab("Subsystems").add(arm.getName(), arm);
    Shuffleboard.getTab("Subsystems").add(claw.getName(), claw);
    Shuffleboard.getTab("Subsystems").add(climb.getName(), climb);
    Shuffleboard.getTab("Subsystems").add(elevator.getName(), elevator);
    // Shuffleboard.getTab("Subsystems").add("Drive Controller", driveController);  TODO: VERIFY REMOVAL
    Shuffleboard.getTab("Subsystems").add("RobotContainer", this);

    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);

    // intakeLocationChooser.addOption("LEFT", "LEFT"); TODO: VERIFY REMOVAL
    // intakeLocationChooser.addOption("RIGHT", "RIGHT"); TODO: VERIFY REMOVAL
    // SmartDashboard.putData(intakeLocationChooser); TODO: VERIFY REMOVAL

    SmartDashboard.putData(autoChooser);
  }

  // public void setDefaultLocation(){ TODO: Verify removal
  //   if(intakeLocationChooser.getSelected() == "LEFT"){
  //       prepStateUtil.setPrepIntakeClimbLocation(INTAKE_CLIMB_LOCATION.LEFT);
  //   }
  //   else{
  //       prepStateUtil.setPrepIntakeClimbLocation(INTAKE_CLIMB_LOCATION.RIGHT);
  //   }
  // }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
    private void configureDriverBindings() { // maybe add ternerary for robot relative based on prep state?
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(driveCommand) 
        );

        Command rumbleBriefly =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                }),
            new InstantCommand(() -> System.out.println("rumble set")),
            new WaitCommand(0.25),
            new InstantCommand(() -> System.out.println("rumble wait ended")),
            new InstantCommand(
                () -> {
                  driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        }));
        
        //INTAKE
        driverController.leftTrigger().whileTrue(
            new IntakeSequenceManual(arm, elevator, claw, preppedIntakeLocation, headingSetter).finallyDo(()->claw.stopMotors())
        );

        //HOME
        driverController.leftBumper().onTrue(
            new NewHomeSequence(arm, elevator, claw)
        );

        //PREP SCORE L3
        driverController.rightBumper().onTrue(
            new PrepScoreAndDrive(elevator, arm, preppedHeight, velocitySetter, headingSetter, desiredHeadingDeg, joystickInput, Constants.LIMELIGHT_FRONT_NAME, MaxSpeed, drivetrain, preppedScoringLocation)
        );

        //SCORE
        driverController.rightTrigger().onTrue(
            new InstantCommand(()->{
                claw.runMotorsScoring();
            })
        );
        driverController.rightTrigger().onFalse(
            new InstantCommand(()->{
                claw.stopMotors();
            })
        );

        driverController.start().onTrue(
            new InstantCommand(
                ()->{
                    drivetrain.seedFieldCentric();
                    desiredHeadingDeg = 0.0;
                }
            )
        );

        // cardinals
        driverController.a().onTrue(
            new InstantCommand(()->this.desiredHeadingDeg = 180.0)
        );
    
        driverController.b().onTrue(
            new InstantCommand(()->this.desiredHeadingDeg = 270.0)
        );
    
        driverController.x().onTrue(
            new InstantCommand(()->this.desiredHeadingDeg = 180.0)
        );
    
        driverController.y().onTrue(
            new InstantCommand(()->this.desiredHeadingDeg = 0.0)
        );
    
        driverController.povRight().onTrue(
            new InstantCommand(()->this.desiredHeadingDeg += 30.0)
        );
    
        driverController.povLeft().onTrue(
            new InstantCommand(()->this.desiredHeadingDeg -= 30.0)
        );

        driverController.povUp().onTrue(
            new PrepScoreManual(elevator, arm, ElevatorHeight.SCORE_L3)
        );
    // TODO
    /* drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-driverController.getLeftY() * MaxSpeed * climb.getThrottle()) // Drive forward with negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed  * climb.getThrottle()) // Drive left with negative X (left)
                .withRotationalRate(-driverController.getRightX() * MaxAngularRate * climb.getThrottle()) // Drive counterclockwise with negative X (left)
        )
    );*/
    
    /* prep score */
    // driverController
    //     .leftBumper()
    //     .onTrue(
    //         new PrepScoreSequence(
    //                 arm,
    //                 elevator,
    //                 scoringLimelight,
    //                 climb,
    //                 prepStateUtil,
    //                 drivetrain,
    //                 matchState,
    //                 lights)
    //             .beforeStarting(
    //                 () -> {
    //                   prepState = PrepState.SCORE;
    //                   System.out.println(prepStateUtil.getPrepScoreHeight().toString());
    //                   driveController.setRobotCentric(true);
    //                 })
    // ); TODO: FIX PrepScoreSequence
    /* climb prep */
    /*driverController
        .back()
        .onTrue(
            new DeepClimbPrep(
                    climb,
                    arm,
                    scoringLimelight,
                    prepStateUtil,
                    matchState,
                    drivetrain,
                    elevator,
                    lights)
                .beforeStarting(() -> prepState = PrepState.CLIMB));*/

    /* intake */

    // driverController.rightTrigger().onTrue(
    // new SequentialCommandGroup(
    //     //new InstantCommand(()-> driveController.setRobotCentric(true)),
    //     new InstantCommand(()->this.desiredHeadingDeg = prepStateUtil.getPrepIntakeClimbLocation()==PrepStateUtil.INTAKE_CLIMB_LOCATION.LEFT?-56.0:56.0)
    // ) );
    // driverController
    //     .rightTrigger()
    //     .whileTrue(
    //         new SequentialCommandGroup(
    //             new InstantCommand(() -> prepState = PrepState.INTAKE),
    //             // new IntakeSequence( TODO: FIX INTAKE SEQUENCE
    //             //     claw, arm, elevator, climb, prepStateUtil, drivetrain, lights),
    //             new InstantCommand(() -> claw.runMotorsIntaking()),
    //             new WaitUntilCommand(claw::beamBreakSeesObject),
    //             new InstantCommand(() -> claw.stopMotors()),
    //             new InstantCommand(() -> prepState = PrepState.OFF),
    //             new ConditionalCommand(
    //                 new InstantCommand(() -> lights.setLEDColor(LightCode.HAS_CORAL)),
    //                 new InstantCommand(),
    //                 claw::beamBreakSeesObject)));
    // driverController
    //     .rightTrigger()
    //     .onFalse(
    //         new SequentialCommandGroup(
    //             new InstantCommand(
    //                 () -> {
    //                   claw.stopMotors();
    //                   prepState = PrepState.OFF;
    //                 }),
    //             new ConditionalCommand(
    //                 new SequentialCommandGroup(
    //                     new WaitCommand(0.5),
    //                     new GoHomeSequenceFake(climb, elevator, arm, claw, lights)),
    //                 new InstantCommand(),
    //                 () -> claw.beamBreakSeesObject())));

    // TreeMap<PrepState, Command> selectCommandMap = new TreeMap<PrepState, Command>();

    // selectCommandMap.put(
    //     PrepState.SCORE,
    //     new SequentialCommandGroup(
    //         new FinishScore(claw, elevator, arm, preppedHeight, lights)
    //         // new InstantCommand(()-> driveController.setRobotCentric(false)) TODO: Verify removal
    //         // new WaitUntilCommand(()->!claw.beamBreakSeesObject()),
    //         // new GoHomeSequenceFake(climb, elevator, arm, claw, lights)
    //         ));
    // selectCommandMap.put(PrepState.CLIMB, new DeepClimbScoringSequence(climb, elevator, lights));

    // SelectCommand<PrepState> driverRightTriggerCommand =
    //     new SelectCommand<PrepState>(
    //         selectCommandMap,
    //         () -> {
    //           PrepState h = prepState;
    //           prepState = PrepState.OFF;
    //           return h;
    //         });

    // /* finish score */
    // driverController
    //     .leftTrigger()
    //     .onTrue(
    //         new ConditionalCommand(
    //                 new InstantCommand(() -> claw.runMotorsScoring()),
    //                 driverRightTriggerCommand,
    //                 () -> prepState == PrepState.OFF)
    //             .beforeStarting(new InstantCommand(() -> System.out.println("right trigger"))));

    // driverController
    //     .leftTrigger()
    //     .onFalse(
    //         new ConditionalCommand(
    //                 new InstantCommand(() -> claw.stopMotors()),
    //                 new InstantCommand(),
    //                 () -> prepState == PrepState.OFF)
    //             .beforeStarting(new InstantCommand(() -> System.out.println("right trigger"))));

    // // driverController.leftTrigger().whileTrue(new InstantCommand(()->claw.runMotorsScoring()));
    // // driverController.leftTrigger().onFalse(new InstantCommand(()->claw.stopMotors()));

    // // driverController
    // // .povRight()
    // // .onTrue(new AlgaeKnockoff(elevator));

    // /*
    //  * povUp = top left back button
    //  * povRight = bottom left back button
    //  * povDown = top right back button
    //  * povLeft = bottom right back button
    //  */

    // /* Go home */
    // driverController
    //     .povLeft()
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             new GoHomeSequence(climb, elevator, arm, claw, lights)
    //             // new InstantCommand(()->driveController.setRobotCentric(false)) TODO: Verify removal
    //         )
    //     );
  }

  private void configureOperatorBindings() {
    operatorController
        .rightTrigger()
        .whileTrue(new InstantCommand(() -> claw.rollerMotor.set(0.2)));
    operatorController.rightTrigger().onFalse(new InstantCommand(() -> claw.rollerMotor.set(0.0)));

    operatorController.povLeft().onTrue(
        new InstantCommand(() -> preppedScoringLocation = PrepScoreAndDrive.Location.LEFT)
    );
    operatorController.povRight().onTrue(
        new InstantCommand(() -> preppedScoringLocation = PrepScoreAndDrive.Location.RIGHT)
    );

    //operatorController.rightBumper().onTrue(new InstantCommand(() -> climb.setServoLocked(true)));
    //operatorController.leftBumper().onTrue(new InstantCommand(() -> climb.setServoLocked(false)));

    operatorController.y().onTrue(
        new InstantCommand(() -> preppedHeight = ElevatorHeight.SCORE_L3)
    );

    operatorController.x().onTrue(
        new InstantCommand(() -> preppedHeight = ElevatorHeight.SCORE_L2)
    );
   
    operatorController.a().onTrue(
        new InstantCommand(() -> preppedHeight = ElevatorHeight.SCORE_L1)
    );
    // operatorController.back().onTrue(new InstantCommand(()->driveController.setRobotCentric(!driveController.robotRelativeActive))); TODO: Verify removal

    operatorController
        .povUp()
        .onTrue(new InstantCommand(() -> preppedIntakeLocation = IntakeSequenceManual.Location.LEFT));
    operatorController
        .povDown()
        .onTrue(new InstantCommand(() -> preppedIntakeLocation = IntakeSequenceManual.Location.RIGHT));

    // operatorController TODO: Verifiy removal
    //     .start()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               // driveController.rezeroControllerAndYawToMsuDefault(); 
    //               // driveController.rezeroControllerAndYawToMsuDefault();
    //               //driveController.rezeroControllerToGyro();
    //               //driveController.rezeroControllerToGyro();
    //             }));

    operatorController.leftTrigger().whileTrue(new InstantCommand(() -> claw.runMotorsOuttake()));
    operatorController.leftTrigger().onFalse(new InstantCommand(() -> claw.stopMotors()));

  }

  // Adds Debug Bindings

  private void configureDebugBindings() {
    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(driveCommand) // Drive counterclockwise with negative X (left)
        );
    // Set Elevator to score_L3
    driverController.rightTrigger().onTrue(
        new InstantCommand(()->elevator.setDesiredPosition(ElevatorHeight.SCORE_L3)));
    // Reset Elevator
    driverController.rightBumper().onTrue(
        new InstantCommand(()->elevator.setDesiredPosition(ElevatorHeight.HOME)));

    // Set Arm postition to L3
    driverController.a().onTrue(
        new InstantCommand(()->arm.setDesiredPosition(ArmPosition.L3)));
    // Reset Arm
    driverController.b().onTrue(
        new InstantCommand(()->arm.setDesiredPosition(ArmPosition.HOME)));
    //set pos to intake
    driverController.x().onTrue(
        new InstantCommand(()->arm.setDesiredPosition(ArmPosition.INTAKE))
    );

    // Intake Claw
    driverController.povUp().onTrue(
        new InstantCommand(()->claw.runMotorsIntaking()));
    // Reset Claw
    driverController.povUp().onFalse(
        new InstantCommand(()->claw.stopMotors()));

    // Set Climb to Climbing
    // driverController.povRight().onTrue(
    //     new InstantCommand(()->climb.setDesiredClimbPosition(ClimbPosition.CLIMBING)));
    // // Reset Climb
    // driverController.povLeft().onTrue(
    //     new InstantCommand(()->climb.setDesiredClimbPosition(ClimbPosition.STOWED)));

    // Set Lights to Party Mode
    driverController.povDown().onTrue(
        new InstantCommand(()->lights.setLEDColor(LightCode.PARTY_MODE)));
    // Reset Lights
    driverController.povDown().onFalse(
        new InstantCommand(()->lights.setLEDColor(LightCode.OFF)));
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
    // super.initSendable(builder);
    builder.addStringProperty("Prep State", () -> prepState.toString(), null);
    builder.addStringProperty("Path CMD", () -> pathCmd, null);
    builder.addStringProperty(
        "Prepped Intake/Climb", () -> prepStateUtil.getPrepIntakeClimbLocation().toString(), null);
    builder.addStringProperty(
        "Prepped Height", () -> prepStateUtil.getPrepScoreHeight().toString(), null);
    builder.addStringProperty(
        "Prepped Scoring Location", () -> prepStateUtil.getPrepScoreLocation().toString(), null);
    builder.addDoubleProperty("PrepStateUtil deg", () -> prepStateUtil.getDegrees(), null);
    builder.addDoubleProperty("odometry X", () -> drivetrain.getState().Pose.getX(), null);
    builder.addDoubleProperty("odometry Y", () -> drivetrain.getState().Pose.getY(), null);
    builder.addDoubleProperty(
        "odometry rotation deg", () -> drivetrain.getState().Pose.getRotation().getDegrees(), null);
    // builder.addBooleanProperty(
    //     "robot relative enabled", () -> driveController.robotRelativeActive, null);
    builder.addDoubleProperty("desired heading deg", () -> this.desiredHeadingDeg, null);
    builder.addDoubleProperty(
        "gyro rotation deg", () -> drivetrain.getPigeon2().getRotation2d().getDegrees(), null);
    builder.addDoubleProperty(
        "broken module encoder 2",
        () -> drivetrain.getModule(2).getEncoder().getAbsolutePosition().getValueAsDouble(),
        null);
    builder.addDoubleProperty(
        "good module encoder 1",
        () -> drivetrain.getModule(1).getEncoder().getAbsolutePosition().getValueAsDouble(),
        null);
    builder.addDoubleProperty(
        "good module encoder 0",
        () -> drivetrain.getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble(),
        null);
    builder.addDoubleProperty(
        "good module encoder 3",
        () -> drivetrain.getModule(3).getEncoder().getAbsolutePosition().getValueAsDouble(),
        null);
    builder.addStringProperty(
        "Current selected auto",
        () -> this.getAutonomousCommand().getName(),
        null);
  }
}
