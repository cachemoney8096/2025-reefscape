// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import frc.robot.subsystems.IntakeLimelight.IntakeLimelight;
import frc.robot.subsystems.ScoringLimelight.ScoringLimelight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.utils.MatchStateUtil;

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
  public DriveSubsystem drive;
  public Elevator elevator;
  public Lights lights;
  public IntakeLimelight intakeLimelight;
  public ScoringLimelight scoringLimelight;

  public String pathCmd = "";

  /* Prep states*/
  public enum IntakeClimbLocation {
    LEFT,
    CENTER,
    RIGHT
  }

  public enum ScoringLocation {
    LEFT,
    RIGHT
  }

  public IntakeClimbLocation preppedLocation = IntakeClimbLocation.LEFT;
  public ElevatorHeight preppedHeight = ElevatorHeight.SCORE_L4;
  public ScoringLocation preppedScoringLocation = ScoringLocation.LEFT;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(MatchStateUtil ms) {
    matchState = ms;

    /* Subsystems */
    arm = new Arm();
    claw = new Claw();
    climb = new Climb();
    drive = new DriveSubsystem();
    elevator = new Elevator();
    lights = new Lights();
    scoringLimelight = new ScoringLimelight(Constants.SCORING_LIMELIGHT_PITCH_DEG, Constants.SCORING_LIMELIGHT_HEIGHT_M, 0.0); //TODO add placeholders in constants
    intakeLimelight = new IntakeLimelight(Constants.INTAKE_LIMELIGHT_PITCH_DEG, Constants.INTAKE_LIMELIGHT_HEIGHT_M, 0.0); //""

    /* Named commands here */

    NamedCommands.registerCommand(
        "AUTO INTAKE SEQUENCE",
        new InstantCommand(() -> pathCmd = "AUTO INTAKE SEQUENCE")
            .andThen(new AutoIntakeSequence(elevator, arm, claw)));

    NamedCommands.registerCommand(
        "AUTO SCORING PREP SEQUENCE",
        new InstantCommand(() -> pathCmd = "AUTO SCORING PREP SEQUENCE")
            .andThen(new AutoScoringPrepSequence(elevator, arm, claw)));

    NamedCommands.registerCommand(
      "AUTO SCORING SEQUENCE",
      new InstantCommand(() -> pathCmd = "AUTO SCORING SEQUENCE")
          .andThen(new AutoScoringSequence(arm, elevator, claw)));

    /* Configure controller bindings */
    configureDriverBindings();
    configureOperatorBindings();

    /* Shuffleboard */
    Shuffleboard.getTab("Subsystems").add(drive.getName(), drive);
    Shuffleboard.getTab("Subsystems").add(arm.getName(), arm);
    Shuffleboard.getTab("Subsystems").add(claw.getName(), claw);
    Shuffleboard.getTab("Subsystems").add(climb.getName(), climb);
    Shuffleboard.getTab("Subsystems").add(elevator.getName(), elevator);

    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);

    /* Autonchooser config */
    autonChooser.setDefaultOption(
        "default option no autos yet",
        new Pair<Command, String>(new InstantCommand(), "PATH NAME"));
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
    /* Go home */
    driverController.leftBumper().onTrue(new GoHomeSequence(climb, elevator, arm, claw, lights));
    /* prep score */
    driverController.rightBumper().onTrue(new PrepScoreSequence(arm, elevator, scoringLimelight, climb, preppedHeight, preppedScoringLocation, drive, matchState));
    /* climb prep */
    driverController.back().onTrue(new DeepClimbPrep(climb, arm, scoringLimelight, preppedLocation, matchState, drive, elevator));
    /* climb */
    driverController.start().onTrue(new DeepClimbScoringSequence(arm, climb));
    /* intake */
    driverController.leftTrigger().whileTrue(new SequentialCommandGroup(new IntakeSequence(claw, intakeLimelight, arm, elevator, climb, preppedLocation, drive), new RunCommand(() -> claw.runMotorsIntaking()).until(claw::beamBreakSeesObject).andThen(() -> {new InstantCommand(() -> claw.stopMotors()); lights.setLEDColor(LightCode.READY_TO_SCORE);})));
    driverController.leftTrigger().onFalse(new InstantCommand(() -> claw.stopMotors()));
    /* finish score */
    driverController.rightTrigger().onTrue(new FinishScore(claw, elevator, arm, preppedHeight));
    /* TODO: CARDINALS */
    /* TODO: DRIVE CODE */
    drive.setDefaultCommand(new InstantCommand());
  }

  private void configureOperatorBindings() {
    /* Left right and center for intake and climb */
    operatorController.povLeft().onTrue(new InstantCommand(()->preppedLocation = IntakeClimbLocation.LEFT));
    operatorController.povUp().onTrue(new InstantCommand(()->preppedLocation = IntakeClimbLocation.CENTER));
    operatorController.povRight().onTrue(new InstantCommand(()->preppedLocation = IntakeClimbLocation.RIGHT));
    /* Height for scoring */
    operatorController.a().onTrue(new InstantCommand(()->preppedHeight = ElevatorHeight.SCORE_L4));
    operatorController.b().onTrue(new InstantCommand(()->preppedHeight = ElevatorHeight.SCORE_L3));
    operatorController.x().onTrue(new InstantCommand(()->preppedHeight = ElevatorHeight.SCORE_L2));
    operatorController.y().onTrue(new InstantCommand(()->preppedHeight = ElevatorHeight.SCORE_L1));
    /* Left and right for scoring */
    operatorController.leftBumper().onTrue(new InstantCommand(()->preppedScoringLocation = ScoringLocation.LEFT));
    operatorController.leftBumper().onTrue(new InstantCommand(()->preppedScoringLocation = ScoringLocation.RIGHT));
    /* TODO: ZERO ROTATION ODOMETRY */
    /* TODO: RESET YAW */
    operatorController.back().onTrue(new InstantCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autonChooser.getSelected().getFirst();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("Prepped Location", () -> preppedLocation.toString(), null);
    builder.addStringProperty("Prepped Height", () -> preppedHeight.toString(), null);
    builder.addStringProperty("Prepped Scoring Location", () -> preppedScoringLocation.toString(), null);

  }
}
