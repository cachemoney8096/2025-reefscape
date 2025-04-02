// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.Units;
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
import frc.robot.commands.AlgaeKnockoff;
import frc.robot.commands.AutoIntakeSequence;
import frc.robot.commands.AutoScoringPrepSequence;
import frc.robot.commands.DeepClimbPrep;
import frc.robot.commands.DeepClimbScoringSequence;
import frc.robot.commands.FinishScore;
import frc.robot.commands.GoHomeSequence;
import frc.robot.commands.GoHomeSequenceFake;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.PrepScoreSequence;
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
import frc.robot.subsystems.IntakeLimelight.IntakeLimelight;
import frc.robot.subsystems.IntakeLimelight.IntakeLimelightConstants;
import frc.robot.subsystems.ScoringLimelight.ScoringLimelight;
import frc.robot.subsystems.ScoringLimelight.ScoringLimelightConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.utils.MatchStateUtil;
import frc.robot.utils.PrepStateUtil;
import frc.robot.utils.PrepStateUtil.INTAKE_CLIMB_LOCATION;
import frc.robot.utils.PrepStateUtil.SCORE_HEIGHT;
import java.util.TreeMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable {
  private MatchStateUtil matchState;

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  /* Pair of the command for an auto and its name */
  private SendableChooser<Pair<Command, String>> autonChooser = new SendableChooser<>();

  private SendableChooser<String> intakeLocationChooser = new SendableChooser<>();

  /* Subsystems */
  public Arm arm;
  public Claw claw;
  public Climb climb;
  // public DriveSubsystem drive;
  public Elevator elevator;
  public Lights lights;
  public IntakeLimelight intakeLimelight;
  public ScoringLimelight scoringLimelight;

  public String pathCmd = "";

  public enum PrepState {
    OFF,
    SCORE,
    CLIMB,
    INTAKE
  }

  /* Prep states */
  public enum IntakeClimbLocation {
    LEFT,
    CENTER,
    RIGHT
  }

  public enum ScoringLocation {
    LEFT,
    RIGHT
  }

  public PrepState prepState = PrepState.OFF;
  public IntakeClimbLocation preppedLocation = IntakeClimbLocation.LEFT;
  // public ElevatorHeight preppedHeight = ElevatorHeight.SCORE_L4;
  public static ElevatorHeight preppedHeight = ElevatorHeight.SCORE_L3;
  public ScoringLocation preppedScoringLocation = ScoringLocation.LEFT;

  /* Garbage from phoenix tuner */
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired
  // top
  // speed
  private double MaxAngularRate =
      Units.RotationsPerSecond.of(2.5).in(Units.RadiansPerSecond); // 3/4 of a
  // rotation
  // per second
  // max angular velocity

  private PrepStateUtil prepStateUtil = new PrepStateUtil();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  // private final SwerveRequest.FieldCentricFacingAngle drive = new
  // SwerveRequest.FieldCentricFacingAngle()
  // .withDeadband(MaxSpeed * 0.1)
  // .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  // .withDriveRequestType(
  // DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.FieldCentricFacingAngle driveWithAngleController =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  // TODO maybe add withDriveRequestType(DriveRequestType.OpenLoopVoltage) if
  // encountering issues?

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain;

  public DriveController driveController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(MatchStateUtil ms) {
    /* Subsystems */
    arm = new Arm();
    claw = new Claw();
    climb = new Climb();
    // drive = new DriveSubsystem(ms);
    driveWithAngleController.HeadingController.setPID(7.0, 0.0, 0); // TODO was 10
    elevator = new Elevator();
    lights = new Lights();
    /* Named commands here */

    SequentialCommandGroup intake = new SequentialCommandGroup(
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
    drivetrain = TunerConstants.createDrivetrain();
    driveController = new DriveController(drivetrain, matchState, driverController);

    scoringLimelight =
        new ScoringLimelight(
            ScoringLimelightConstants.SCORING_LIMELIGHT_PITCH_DEGREES,
            ScoringLimelightConstants.SCORING_LIMELIGHT_HEIGHT_METERS,
            0.0);
    intakeLimelight =
        new IntakeLimelight(
            IntakeLimelightConstants.INTAKE_LIMELIGHT_PITCH_DEGREES,
            IntakeLimelightConstants.INTAKE_LIMELIGHT_HEIGHT_METERS,
            0.0); // ""

    drivetrain.registerTelemetry(logger::telemeterize);

    /* Configure controller bindings */
    configureDriverBindings();
    configureOperatorBindings();

    /* Shuffleboard */
    // Shuffleboard.getTab("Subsystems").add(drivetrain.getName(), drive);
    Shuffleboard.getTab("Subsystems").add(arm.getName(), arm);
    Shuffleboard.getTab("Subsystems").add(claw.getName(), claw);
    Shuffleboard.getTab("Subsystems").add(climb.getName(), climb);
    Shuffleboard.getTab("Subsystems").add(elevator.getName(), elevator);
    Shuffleboard.getTab("Subsystems").add("Drive Controller", driveController);
    Shuffleboard.getTab("Subsystems").add("RobotContainer", this);

    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);

    intakeLocationChooser.addOption("LEFT", "LEFT");
    intakeLocationChooser.addOption("RIGHT", "RIGHT");
    SmartDashboard.putData(intakeLocationChooser);

    /* Autonchooser config */
    // scoring location 1
    autonChooser.addOption(
        "P2_S1_I_J",
        new Pair<Command, String>(
            new P2_S1_I_J(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));

    autonChooser.addOption(
        "P2_S1_J_I",
        new Pair<Command, String>(
            new P2_S1_J_I(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));

    autonChooser.addOption(
        "P3_S1_I_J_K",
        new Pair<Command, String>(
            new P3_S1_I_J_K(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));

    autonChooser.addOption(
        "P3_S1_J_I_K",
        new Pair<Command, String>(
            new P3_S1_J_I_K(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));

    autonChooser.addOption(
        "P4_S1_I_J_K_L",
        new Pair<Command, String>(
            new P4_S1_I_J_K_L(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));

    autonChooser.addOption(
        "P4_S1_J_I_K_L",
        new Pair<Command, String>(
            new P4_S1_J_I_K_L(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));
    // starting location 2
    autonChooser.addOption(
        "P1_S2_G",
        new Pair<Command, String>(
            new P1_S2_G(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));
    autonChooser.addOption(
        "P1_S2_H",
        new Pair<Command, String>(
            new P1_S2_H(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));
    autonChooser.addOption(
        "P2_S2_G_H",
        new Pair<Command, String>(
            new P2_S2_G_H(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));
    autonChooser.addOption(
        "P2_S2_H_G",
        new Pair<Command, String>(
            new P2_S2_H_G(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));

    autonChooser.addOption(
        "P3_S2_G_H_B",
        new Pair<Command, String>(
            new P3_S2_G_H_B(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));
    autonChooser.addOption(
        "P3_S2_H_G_A",
        new Pair<Command, String>(
            new P3_S2_H_G_A(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));
    // starting location 3
    autonChooser.addOption(
        "P2_S3_F_E",
        new Pair<Command, String>(
            new P2_S3_F_E(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));
    autonChooser.addOption(
        "P3_S3_E_F_D",
        new Pair<Command, String>(
            new P3_S3_E_F_D(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));
    autonChooser.addOption(
        "P3_S3_F_E_D",
        new Pair<Command, String>(
            new P3_S3_F_E_D(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));
    autonChooser.addOption(
        "P4_S3_E_F_D_C",
        new Pair<Command, String>(
            new P4_S3_E_F_D_C(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));
    autonChooser.addOption(
        "P4_S3_F_E_D_C",
        new Pair<Command, String>(
            new P4_S3_F_E_D_C(
                drivetrain,
                arm,
                claw,
                elevator,
                intakeLimelight,
                scoringLimelight,
                matchState.isRed()),
            null));

    SmartDashboard.putData(autonChooser);
  }

  public void setDefaultLocation(){
    if(intakeLocationChooser.getSelected() == "LEFT"){
        prepStateUtil.setPrepIntakeClimbLocation(INTAKE_CLIMB_LOCATION.LEFT);
    }
    else{
        prepStateUtil.setPrepIntakeClimbLocation(INTAKE_CLIMB_LOCATION.RIGHT);
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void
      configureDriverBindings() { // maybe add ternerary for robot relative based on prep state?
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
    driverController
        .leftBumper()
        .onTrue(
            new PrepScoreSequence(
                    arm,
                    elevator,
                    scoringLimelight,
                    climb,
                    prepStateUtil,
                    drivetrain,
                    matchState,
                    lights)
                .beforeStarting(
                    () -> {
                      prepState = PrepState.SCORE;
                      System.out.println(prepStateUtil.getPrepScoreHeight().toString());
                      driveController.setRobotCentric(true);
                    })
    );
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
    /* intake */

    boolean blue = matchState.isBlue();

    driverController.rightTrigger().onTrue(new InstantCommand(()-> driveController.setRobotCentric(true)));
    driverController
        .rightTrigger()
        .whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> prepState = PrepState.INTAKE),
                new InstantCommand(()->driveController.setDesiredHeading(prepStateUtil.getPrepIntakeClimbLocation()==PrepStateUtil.INTAKE_CLIMB_LOCATION.LEFT?(blue?-56.0:236):(blue?56.0:124.0))),
                new IntakeSequence(
                    claw, intakeLimelight, arm, elevator, climb, prepStateUtil, drivetrain, lights),
                new InstantCommand(() -> claw.runMotorsIntaking()),
                new WaitUntilCommand(claw::beamBreakSeesObject),
                new InstantCommand(() -> claw.stopMotors()),
                new InstantCommand(() -> prepState = PrepState.OFF),
                new ConditionalCommand(
                    new InstantCommand(() -> lights.setLEDColor(LightCode.HAS_CORAL)),
                    new InstantCommand(),
                    claw::beamBreakSeesObject)));
    driverController
        .rightTrigger()
        .onFalse(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> {
                      claw.stopMotors();
                      prepState = PrepState.OFF;
                    }),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        new GoHomeSequenceFake(climb, elevator, arm, claw, lights)),
                    new InstantCommand(),
                    () -> claw.beamBreakSeesObject())));

    TreeMap<PrepState, Command> selectCommandMap = new TreeMap<PrepState, Command>();

    selectCommandMap.put(
        PrepState.SCORE,
        new SequentialCommandGroup(
            new FinishScore(claw, elevator, arm, preppedHeight, lights),
            new InstantCommand(()-> driveController.setRobotCentric(false))
            // new WaitUntilCommand(()->!claw.beamBreakSeesObject()),
            // new GoHomeSequenceFake(climb, elevator, arm, claw, lights)
            ));
    selectCommandMap.put(PrepState.CLIMB, new DeepClimbScoringSequence(climb, elevator, lights));

    SelectCommand<PrepState> driverRightTriggerCommand =
        new SelectCommand<PrepState>(
            selectCommandMap,
            () -> {
              PrepState h = prepState;
              prepState = PrepState.OFF;
              return h;
            });

    /* finish score */
    driverController
        .leftTrigger()
        .onTrue(
            new ConditionalCommand(
                    new InstantCommand(() -> claw.runMotorsScoring()),
                    driverRightTriggerCommand,
                    () -> prepState == PrepState.OFF)
                .beforeStarting(new InstantCommand(() -> System.out.println("right trigger"))));

    driverController
        .leftTrigger()
        .onFalse(
            new ConditionalCommand(
                    new InstantCommand(() -> claw.stopMotors()),
                    new InstantCommand(),
                    () -> prepState == PrepState.OFF)
                .beforeStarting(new InstantCommand(() -> System.out.println("right trigger"))));

    // driverController.leftTrigger().whileTrue(new InstantCommand(()->claw.runMotorsScoring()));
    // driverController.leftTrigger().onFalse(new InstantCommand(()->claw.stopMotors()));

    driverController
        .y()
        .onTrue(
            new InstantCommand(
                () -> driveController.setDesiredHeading(matchState.isBlue() ? 0 : 180)));
    driverController
        .b()
        .onTrue(
            new InstantCommand(
                () -> driveController.setDesiredHeading(matchState.isBlue() ? 270 : 90)));
    driverController
        .a()
        .onTrue(
            new InstantCommand(
                () -> driveController.setDesiredHeading(matchState.isBlue() ? 180 : 0)));
    driverController
        .x()
        .onTrue(
            new InstantCommand(
                () -> driveController.setDesiredHeading(matchState.isBlue() ? 90 : 270)));
    driverController
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> driveController.setDesiredHeading(driveController.getDesiredHeading() + 30)));
    driverController
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> driveController.setDesiredHeading(driveController.getDesiredHeading() - 30)));

    // driverController
    // .povRight()
    // .onTrue(new AlgaeKnockoff(elevator));

    /*
     * povUp = top left back button
     * povRight = bottom left back button
     * povDown = top right back button
     * povLeft = bottom right back button
     */

    /* Go home */
    driverController
        .povLeft()
        .onTrue(
            new ParallelCommandGroup(
                new GoHomeSequence(climb, elevator, arm, claw, lights),
                new InstantCommand(()->driveController.setRobotCentric(false))
            )
        );
  }

  private void configureOperatorBindings() {
    operatorController
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                () -> driveController.setRobotCentric(!driveController.robotRelativeActive)));

    operatorController
        .rightTrigger()
        .whileTrue(new InstantCommand(() -> claw.rollerMotor.set(0.2)));
    operatorController.rightTrigger().onFalse(new InstantCommand(() -> claw.rollerMotor.set(0.0)));

    operatorController
        .povUp()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> elevator.setDesiredPosition(ElevatorHeight.ARM_CLEAR_OF_CLIMB)),
                new WaitUntilCommand(elevator::atDesiredPosition),
                new InstantCommand(() -> {
                    climb.setDesiredClimbPosition(ClimbPosition.CLIMBING_PREP);
                    arm.setDesiredPosition(ArmPosition.DEEP_CLIMB);
                    driveController.setRobotCentric(true);
                })

            ));
                /** 
                 * () -> {
                  elevator.setDesiredPosition(ElevatorHeight.ARM_CLEAR_OF_CLIMB);
                  
                  climb.setDesiredClimbPosition(ClimbPosition.CLIMBING_PREP);
                  arm.setDesiredPosition(ArmPosition.DEEP_CLIMB);
                  driveController.setRobotCentric(true);
                })
                 */
    operatorController
        .povDown()
        .onTrue(new InstantCommand(() -> climb.setDesiredClimbPosition(ClimbPosition.CLIMBING)));
    operatorController
        .povLeft()
        .onTrue(new InstantCommand(() -> climb.setDesiredClimbPosition(ClimbPosition.STOWED)));
    //operatorController.povRight().onTrue(new InstantCommand(() -> climb.stopClimbMovement()));
    operatorController.povRight().onTrue(new InstantCommand(()->climb.bringClimbInFiveDegrees()));

    operatorController.rightBumper().onTrue(new InstantCommand(() -> climb.setServoLocked(true)));
    operatorController.leftBumper().onTrue(new InstantCommand(() -> climb.setServoLocked(false)));

    //operatorController.leftBumper().onTrue(new InstantCommand(()->prepStateUtil.setPrepIntakeClimbLocation(INTAKE_CLIMB_LOCATION.LEFT)));
    //operatorController.rightBumper().onTrue(new InstantCommand(()->prepStateUtil.setPrepIntakeClimbLocation(INTAKE_CLIMB_LOCATION.RIGHT)));


    operatorController.a().onTrue(new AlgaeKnockoff(elevator));

    operatorController.back().onTrue(new InstantCommand(()->driveController.setRobotCentric(!driveController.robotRelativeActive)));

    operatorController
        .y()
        .onTrue(new InstantCommand(() -> prepStateUtil.setPrepScoreHeight(SCORE_HEIGHT.L1)));
    operatorController
        .x()
        .onTrue(new InstantCommand(() -> prepStateUtil.setPrepScoreHeight(SCORE_HEIGHT.L2)));
    operatorController
        .b()
        .onTrue(new InstantCommand(() -> prepStateUtil.setPrepScoreHeight(SCORE_HEIGHT.L3)));

    operatorController
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                  driveController.rezeroControllerAndYawToMsuDefault();
                  driveController.rezeroControllerAndYawToMsuDefault();
                }));

    operatorController.leftTrigger().whileTrue(new InstantCommand(() -> claw.runMotorsOuttake()));
    operatorController.leftTrigger().onFalse(new InstantCommand(() -> claw.stopMotors()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected().getFirst();
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
    builder.addBooleanProperty(
        "robot relative enabled", () -> driveController.robotRelativeActive, null);
    builder.addDoubleProperty("desired heading", () -> driveController.getDesiredHeading(), null);
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
  }
}
