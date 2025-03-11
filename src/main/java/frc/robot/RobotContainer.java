// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoIntakeSequence;
import frc.robot.commands.AutoScoringPrepSequence;
import frc.robot.commands.AutoScoringSequence;
import frc.robot.commands.DeepClimbPrep;
import frc.robot.commands.DeepClimbScoringSequence;
import frc.robot.commands.FinishScore;
import frc.robot.commands.GoHomeSequence;
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
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.utils.MatchStateUtil;
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
    CLIMB
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
  public ElevatorHeight preppedHeight = ElevatorHeight.SCORE_L4;
  public ScoringLocation preppedScoringLocation = ScoringLocation.LEFT;

  /* Garbage from phoenix tuner */
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second
  // max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(MatchStateUtil ms) {
    matchState = ms;

    /* Subsystems */
    arm = new Arm();
    claw = new Claw();
    climb = new Climb();
    // drive = new DriveSubsystem(ms);
    elevator = new Elevator();
    lights = new Lights();
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

    /* Named commands here */

    NamedCommands.registerCommand(
        "AUTO INTAKE SEQUENCE",
        new InstantCommand(() -> pathCmd = "AUTO INTAKE SEQUENCE")
            .andThen(new AutoIntakeSequence(elevator, arm, claw, lights)));

    NamedCommands.registerCommand(
        "AUTO SCORING PREP SEQUENCE",
        new InstantCommand(() -> pathCmd = "AUTO SCORING PREP SEQUENCE")
            .andThen(new AutoScoringPrepSequence(elevator, arm, claw, lights)));

    NamedCommands.registerCommand(
        "AUTO SCORING SEQUENCE",
        new InstantCommand(() -> pathCmd = "AUTO SCORING SEQUENCE")
            .andThen(new AutoScoringSequence(arm, elevator, claw)));

    /* Configure controller bindings */
    configureDriverBindings();
    configureOperatorBindings();

    /* Shuffleboard */
    //Shuffleboard.getTab("Subsystems").add(drivetrain.getName(), drive);
    Shuffleboard.getTab("Subsystems").add(arm.getName(), arm);
    Shuffleboard.getTab("Subsystems").add(claw.getName(), claw);
    Shuffleboard.getTab("Subsystems").add(climb.getName(), climb);
    Shuffleboard.getTab("Subsystems").add(elevator.getName(), elevator);

    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);

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

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDriverBindings() {
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -driverController.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -driverController.getLeftX()
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -driverController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));
    /* prep score */
    driverController
        .rightBumper()
        .onTrue(
            new PrepScoreSequence(
                    arm,
                    elevator,
                    scoringLimelight,
                    climb,
                    preppedHeight,
                    preppedScoringLocation,
                    drivetrain,
                    matchState,
                    lights)
                .beforeStarting(() -> prepState = PrepState.SCORE));
    /* climb prep */
    driverController
        .back()
        .onTrue(
            new DeepClimbPrep(
                    climb,
                    arm,
                    scoringLimelight,
                    preppedLocation,
                    matchState,
                    drivetrain,
                    elevator,
                    lights)
                .beforeStarting(() -> prepState = PrepState.CLIMB));
    /* intake */
    driverController
        .leftTrigger()
        .whileTrue(
            new SequentialCommandGroup(
                new IntakeSequence(
                    claw,
                    intakeLimelight,
                    arm,
                    elevator,
                    climb,
                    preppedLocation,
                    drivetrain,
                    lights),
                new InstantCommand(() -> lights.setLEDColor(LightCode.READY_TO_INTAKE)),
                new RunCommand(() -> claw.runMotorsIntaking())
                    .until(claw::beamBreakSeesObject)
                    .andThen(
                        () -> {
                          new InstantCommand(() -> claw.stopMotors());
                          new ConditionalCommand(
                              new InstantCommand(() -> lights.setLEDColor(LightCode.HAS_CORAL)),
                              new InstantCommand(),
                              claw::beamBreakSeesObject);
                        })));
    driverController.leftTrigger().onFalse(new InstantCommand(() -> claw.stopMotors()));

    TreeMap<PrepState, Command> selectCommandMap = new TreeMap<PrepState, Command>();

    selectCommandMap.put(
        PrepState.SCORE, new FinishScore(claw, elevator, arm, preppedHeight, lights));
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
        .rightTrigger()
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(() -> claw.runMotorsScoring()),
                driverRightTriggerCommand,
                () -> prepState == PrepState.OFF));

    driverController
        .rightTrigger()
        .onFalse(
            new ConditionalCommand(
                new InstantCommand(() -> claw.stopMotors()),
                new InstantCommand(),
                () -> prepState == PrepState.OFF));

    // TODO check these
    driverController
        .y()
        .onTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        Rotation2d.fromDegrees(matchState.isBlue() ? 0 : 180))));
    driverController
        .b()
        .onTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        Rotation2d.fromDegrees(matchState.isBlue() ? 90 : 270))));
    driverController
        .a()
        .onTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        Rotation2d.fromDegrees(matchState.isBlue() ? 180 : 0))));
    driverController
        .x()
        .onTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        Rotation2d.fromDegrees(matchState.isBlue() ? 270 : 90))));

    /* TODO: CHANGE BINDINGS LATER */

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
            new GoHomeSequence(climb, elevator, arm, claw, lights)
                .beforeStarting(
                    () -> {
                      prepState = PrepState.OFF;
                      drivetrain.killDriveToPose();
                    }));
  }

  private void configureOperatorBindings() {
    /* Left right and center for intake and climb */
    /*operatorController
        .povLeft()
        .onTrue(new InstantCommand(() -> preppedLocation = IntakeClimbLocation.LEFT));
    operatorController
        .povUp()
        .onTrue(new InstantCommand(() -> preppedLocation = IntakeClimbLocation.CENTER));
    operatorController
        .povRight()
        .onTrue(new InstantCommand(() -> preppedLocation = IntakeClimbLocation.RIGHT));
    /* Height for scoring */
    /*operatorController
        .a()
        .onTrue(new InstantCommand(() -> preppedHeight = ElevatorHeight.SCORE_L4));
    operatorController
        .b()
        .onTrue(new InstantCommand(() -> preppedHeight = ElevatorHeight.SCORE_L3));
    operatorController
        .x()
        .onTrue(new InstantCommand(() -> preppedHeight = ElevatorHeight.SCORE_L2));
    operatorController
        .y()
        .onTrue(new InstantCommand(() -> preppedHeight = ElevatorHeight.SCORE_L1));

    /* Left and right for scoring */
    /*operatorController
        .leftBumper()
        .onTrue(new InstantCommand(() -> preppedScoringLocation = ScoringLocation.LEFT));
    operatorController
        .rightBumper()
        .onTrue(new InstantCommand(() -> preppedScoringLocation = ScoringLocation.RIGHT));
    new Rotation2d();
    */
     //Testing code for climb, arm, and elevator
     /*operatorController
     .y()
     .whileTrue(new InstantCommand(() -> arm.testArmMovementUp()));
      operatorController
      .y()
      .onFalse(new InstantCommand(() -> arm.stopArmMovement()));
      operatorController
      .a()
      .whileTrue(new InstantCommand(() -> arm.testArmMovementDown()));
      operatorController
      .a()
      .onFalse(new InstantCommand(() -> arm.stopArmMovement()));
      operatorController
      .povUp()
      .whileTrue(new InstantCommand(() -> climb.testClimbMovementUp()));
      operatorController
      .povUp()
     .onFalse(new InstantCommand(() -> climb.stopClimbMovement()));
      operatorController
      .povDown()
      .whileTrue(new InstantCommand(() -> climb.testClimbMovementDown()));
      operatorController
      .povDown()
      .onFalse(new InstantCommand(() -> climb.stopClimbMovement()));*/
      /*operatorController //these are backwards for up and down
      .povRight()
      .whileTrue(new InstantCommand(() -> elevator.testElevatorMovementUp()));*/
      /*
      operatorController
      .povRight()
      .onFalse(new InstantCommand(() -> elevator.stopElevatorMovement()));
      operatorController
      .povLeft()
      .whileTrue(new InstantCommand(() -> elevator.testElevatorMovementDown()));
      operatorController
      .povLeft()
      .onFalse(new InstantCommand(() -> elevator.stopElevatorMovement()));*/
      operatorController.povRight().onTrue(new InstantCommand(()->elevator.setDesiredPosition(ElevatorHeight.SCORE_L3)));
      operatorController.povLeft().onTrue(new InstantCommand(()->elevator.setDesiredPosition(ElevatorHeight.HOME)));

      operatorController.povUp().onTrue(new InstantCommand(()->arm.setDesiredPosition(ArmPosition.INTAKE)));
      operatorController.povDown().onTrue(new InstantCommand(()->arm.setDesiredPosition(ArmPosition.HOME)));
      operatorController.a().onTrue(new InstantCommand(()->climb.setDesiredClimbPosition(ClimbPosition.STOWED)));
      operatorController.b().onTrue(new InstantCommand(()->climb.setDesiredClimbPosition(ClimbPosition.CLIMBING)));
      //operatorController.x().onTrue(new InstantCommand(()->claw.runMotorsIntaking()));
      operatorController.x().onTrue(new InstantCommand(()->claw.rollerMotor.setVoltage(-8.0)));
      operatorController.y().onTrue(new InstantCommand(()->claw.rollerMotor.setVoltage(0.0)));
    operatorController
        .start()
        .onTrue(
            new InstantCommand(
                () ->
                    drivetrain.resetRotation(
                        Rotation2d.fromDegrees(matchState.isBlue() ? 0 : 180))));

    operatorController.back().whileTrue(new RunCommand(() -> claw.runMotorsOuttake(), claw));
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
    builder.addStringProperty("Prep State", () -> prepState.toString(), null);
    builder.addStringProperty("Prepped Location", () -> preppedLocation.toString(), null);
    builder.addStringProperty("Prepped Height", () -> preppedHeight.toString(), null);
    builder.addStringProperty(
        "Prepped Scoring Location", () -> preppedScoringLocation.toString(), null);
    builder.addDoubleProperty("runNumber", ()->1.0, null);
  }
}
