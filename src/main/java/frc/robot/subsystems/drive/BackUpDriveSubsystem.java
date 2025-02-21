package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.GeometryUtils;
import frc.robot.utils.MatchStateUtil;
import frc.robot.utils.PoseBuffer;
import java.util.List;
import java.util.Optional;

public class BackUpDriveSubsystem extends SubsystemBase {

  private double targetHeadingDegrees;

  double tagTargetHeading = 0.0;

  // Create SwerveModules
  public final SwerveModule frontLeft = new SwerveModule(
      RobotMap.FRONT_LEFT_DRIVING_CAN_ID,
      RobotMap.FRONT_LEFT_TURNING_CAN_ID,
      DriveCal.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule frontRight = new SwerveModule(
      RobotMap.FRONT_RIGHT_DRIVING_CAN_ID,
      RobotMap.FRONT_RIGHT_TURNING_CAN_ID,
      DriveCal.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule rearLeft = new SwerveModule(
      RobotMap.REAR_LEFT_DRIVING_CAN_ID,
      RobotMap.REAR_LEFT_TURNING_CAN_ID,
      DriveCal.BACK_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule rearRight = new SwerveModule(
      RobotMap.REAR_RIGHT_DRIVING_CAN_ID,
      RobotMap.REAR_RIGHT_TURNING_CAN_ID,
      DriveCal.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  private final Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON_CAN_ID);
  private ChassisSpeeds lastSetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  public Optional<Pose2d> targetPose = Optional.empty();

  public PoseBuffer poseBuffer = new PoseBuffer();

  double KeepHeadingPID = 0.0;
  double KeepHeadingFF = 0.0;

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(Constants.PLACEHOLDER_DOUBLE),
      getModulePositions());

  public double throttleMultiplier = 1.0;

  private double rotControllerInput = 0.0;

  public MatchStateUtil matchState;

  private InterpolatingDoubleTreeMap yawOffsetMap;

  private InterpolatingDoubleTreeMap velocityToMultiplierMap;

  public BackUpDriveSubsystem(MatchStateUtil matchState) {
    intializeGyro();
    this.matchState = matchState;

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getCurrentChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> setOutputRobotRelativeSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new PPHolonomicDriveController(DriveCal.PATH_TRANSLATION_CONTROLLER, DriveCal.PATH_ROTATION_CONTROLLER),
        config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    yawOffsetMap = new InterpolatingDoubleTreeMap();
    //TODO: TUNE THESE VALUES
    yawOffsetMap.put(0.0, 0.0);
    yawOffsetMap.put(120.0, 5.0);
    yawOffsetMap.put(167.0, 22.0);
    yawOffsetMap.put(330.0, 50.0);

    velocityToMultiplierMap = new InterpolatingDoubleTreeMap();
    velocityToMultiplierMap.put(0.0, DriveCal.MIN_ROTATE_TO_TARGET_PID_OUTPUT);
    velocityToMultiplierMap.put(DriveConstants.MAX_SPEED_METERS_PER_SECOND, 1.0);

    SmartDashboard.putNumber("Norm Velocity (mps)", 0);
    SmartDashboard.putNumber("Velocity PID multiplier", 0);
  }

  public void intializeGyro() {
    // Reset to defaults
    Pigeon2Configurator cfg = gyro.getConfigurator();
    Pigeon2Configuration blankGyroConfiguration = new Pigeon2Configuration();
    cfg.apply(blankGyroConfiguration);
    final double fastUpdateFrequencyHz = 50.0; // todo update for better odometry
    gyro.getYaw().setUpdateFrequency(fastUpdateFrequencyHz);
    gyro.getAngularVelocityZWorld().setUpdateFrequency(fastUpdateFrequencyHz);

    // Set mount pose
    MountPoseConfigs gyroConfig = new MountPoseConfigs();
    gyroConfig.MountPosePitch = 0;
    gyroConfig.MountPoseRoll = 0;
    gyroConfig.MountPoseYaw = 90;
    cfg.apply(gyroConfig);

    // Reset position to zero, this may be overwritten by a path at the start of
    // auto
    gyro.reset();
    Timer.delay(0.1);
    gyro.reset();
  }

  public Pigeon2 getGyro() {
    return gyro;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    frontLeft.periodic();
    frontRight.periodic();
    rearLeft.periodic();
    rearRight.periodic();
    odometry.update(Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()), getModulePositions());
    poseBuffer.pushToBuffer(getPose(), Timer.getFPGATimestamp());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition()
    };
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getCurrentChassisSpeeds() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState());
  }

  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    Pose2d futureRobotPose = new Pose2d(
        originalSpeeds.vxMetersPerSecond * DriveConstants.LOOP_TIME_SEC,
        originalSpeeds.vyMetersPerSecond * DriveConstants.LOOP_TIME_SEC,
        Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * DriveConstants.LOOP_TIME_SEC));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
        twistForPose.dx / DriveConstants.LOOP_TIME_SEC,
        twistForPose.dy / DriveConstants.LOOP_TIME_SEC,
        twistForPose.dtheta / DriveConstants.LOOP_TIME_SEC);
    return updatedSpeeds;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0], false);
    frontRight.setDesiredState(desiredStates[1], false);
    rearLeft.setDesiredState(desiredStates[2], false);
    rearRight.setDesiredState(desiredStates[3], false);
  }

  public void setOutputRobotRelativeSpeeds(ChassisSpeeds desiredChassisSpeeds) {

    ChassisSpeeds correctedDesiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

    lastSetChassisSpeeds = correctedDesiredChassisSpeeds;
    var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);
    setModuleStates(swerveModuleStates);
  }

  public void resetYawToAngle(double yawDeg) {
    double curYawDeg = gyro.getYaw().getValueAsDouble();
    double offsetToTargetDeg = targetHeadingDegrees - curYawDeg;
    gyro.setYaw(yawDeg);
    Pose2d curPose = getPose();
    Pose2d resetPose = new Pose2d(curPose.getTranslation(), Rotation2d.fromDegrees(yawDeg));
    odometry.resetPosition(Rotation2d.fromDegrees(yawDeg), getModulePositions(), resetPose);
    targetHeadingDegrees = yawDeg + offsetToTargetDeg;
  }

  public void resetYaw() {
    resetYawToAngle(matchState.isBlue() ? 0 : 180);
  }

  public void setNoMove() {
    Rotation2d frontLeftCurrRot = frontLeft.getPosition().angle;
    Rotation2d frontRightCurrRot = frontRight.getPosition().angle;
    Rotation2d rearLeftCurrRot = rearLeft.getPosition().angle;
    Rotation2d rearRightCurrRot = rearRight.getPosition().angle;
    frontLeft.setDesiredState(new SwerveModuleState(0, frontLeftCurrRot), true);
    frontRight.setDesiredState(new SwerveModuleState(0, frontRightCurrRot), true);
    rearLeft.setDesiredState(new SwerveModuleState(0, rearLeftCurrRot), true);
    rearRight.setDesiredState(new SwerveModuleState(0, rearRightCurrRot), true);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (xSpeed == 0 && ySpeed == 0 && rot == 0) {
      setNoMove();
      return;
    }
    xSpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    ySpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    rot *= DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SECONDS;

    xSpeed *= throttleMultiplier;
    ySpeed *= throttleMultiplier;
    rot *= throttleMultiplier;

    ChassisSpeeds desiredChassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()))
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
    lastSetChassisSpeeds = desiredChassisSpeeds;

    var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);
    setModuleStates(swerveModuleStates);
  }

  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
  }

  public void resetEncoders() {
    frontLeft.resetDriveEncoder();
    rearLeft.resetDriveEncoder();
    frontRight.resetDriveEncoder();
    rearRight.resetDriveEncoder();
  }

  public double getHeadingDegrees() {
    return gyro.getYaw().getValueAsDouble();
  }

  public double getTurnRate() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  public void keepHeading(double x, double y, boolean fieldRelative) {
    double currentHeadingDegrees = getHeadingDegrees();
    double headingDifferenceDegrees = currentHeadingDegrees - targetHeadingDegrees;
    double offsetHeadingDegrees = MathUtil.inputModulus(headingDifferenceDegrees, -180, 180);

    double pidRotation = DriveCal.ROTATE_TO_TARGET_PID_CONTROLLER.calculate(offsetHeadingDegrees, 0.0);
    double ffRotation = Math.signum(offsetHeadingDegrees) * DriveCal.ROTATE_TO_TARGET_FF;

    double normVelocity = new Translation2d(
        lastSetChassisSpeeds.vxMetersPerSecond, lastSetChassisSpeeds.vyMetersPerSecond)
        .getNorm();

    SmartDashboard.putNumber("Norm Velocity (mps)", normVelocity);

    double velocityMultiplier = velocityToMultiplierMap.get(normVelocity);

    SmartDashboard.putNumber("Velocity PID multiplier", velocityMultiplier);

    pidRotation *= velocityMultiplier;

    KeepHeadingPID = pidRotation;
    KeepHeadingFF = ffRotation;

    double desiredRotation = pidRotation - ffRotation;

    if (Math.abs(desiredRotation) < DriveCal.ROTATION_DEADBAND_THRESHOLD) {
      desiredRotation = 0;
    }

    desiredRotation = MathUtil.clamp(desiredRotation, -1.0, 1.0);

    drive(x, y, desiredRotation, fieldRelative);
  }

  /**
   * Rotates to a specific angle from the D-pad buttons
   *
   * @param povAngleDeg Angle of D-pad control
   */
  public int convertCardinalDirections(int povAngleDeg) {
    if (povAngleDeg == 270) {
      povAngleDeg += 0;
    } else if (povAngleDeg == 90) {
      povAngleDeg -= 0;
    }
    // targetHeadingDegrees is counterclockwise so need to flip povAngle
    povAngleDeg = 360 - povAngleDeg;
    return povAngleDeg;
  }

  public void rotateOrKeepHeading(double x, double y, double rot, boolean fieldRelative, int cardinalAngleDeg) {
    rotControllerInput = rot;
    if (cardinalAngleDeg != -1) {
      targetHeadingDegrees = convertCardinalDirections(cardinalAngleDeg);
      keepHeading(x, y, fieldRelative);
    } else if (rot == 0) {
      keepHeading(x, y, fieldRelative);
    } else {
      targetHeadingDegrees = getHeadingDegrees()
          + calculateYawOffsetDeg(
              Units.radiansToDegrees(lastSetChassisSpeeds.omegaRadiansPerSecond));
      drive(x, y, rot, fieldRelative);
    }
  }

  private double calculateYawOffsetDeg(double rotationalVelocityDeg) {
    double posRotationalVelocityDeg = Math.abs(rotationalVelocityDeg);
    double posOffsetDeg = yawOffsetMap.get(posRotationalVelocityDeg);
    return posOffsetDeg * Math.signum(lastSetChassisSpeeds.omegaRadiansPerSecond);
  }

  public Command followTrajectoryCommand(PathPlannerPath path, boolean isFirstPath) {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      return new SequentialCommandGroup(
          new InstantCommand(
              () -> {
                // Reset odometry for the first path you run during auto
                if (isFirstPath) {
                  this.resetOdometry(path.getStartingHolonomicPose().get());
                }
              }),
          new InstantCommand(
              () -> {
                targetHeadingDegrees = getHeadingDegrees();
              }),
          new FollowPathCommand(
              path,
              this::getPose,
              this::getCurrentChassisSpeeds,
              (speeds, feedforwards) -> setOutputRobotRelativeSpeeds(speeds),
              DriveCal.DRIVE_CONTROLLER,
              config,
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this),
          new PrintCommand("Finished a trajectory"));
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      return new SequentialCommandGroup(null);
    }
  }

  public Pose2d getPastBufferedPose(double latencySec) {
    Optional<Pose2d> p = poseBuffer.getPoseAtTimestamp(Timer.getFPGATimestamp() - latencySec);
    if (!p.isPresent()) {
      return extrapolatePastPoseBasedOnVelocity(Timer.getFPGATimestamp() - latencySec);
    }
    return p.get();
  }

  public Pose2d extrapolatePastPoseBasedOnVelocity(double latencySec) {
    Pose2d curPose = getPose();
    double latencyAdjustmentSec = 0.00;
    latencySec += latencyAdjustmentSec;
    Transform2d pastTransform = new Transform2d(
        new Translation2d(
            -lastSetChassisSpeeds.vxMetersPerSecond * latencySec,
            -lastSetChassisSpeeds.vyMetersPerSecond * latencySec),
        Rotation2d.fromRadians(lastSetChassisSpeeds.omegaRadiansPerSecond * latencySec)
            .unaryMinus());
    Pose2d pastPose = curPose.plus(pastTransform);
    return pastPose;
  }

  public void setLimelightTargetFromTransform(
      Transform2d transform, double latencySec, boolean usingFrontLimelight) {
    // Transform is to get the limelight to the correct location, not to get the
    // robot
    // Here we correct for that
    Transform2d flipTransform = new Transform2d(
        new Translation2d(
            usingFrontLimelight ? (transform.getX()) : (-transform.getX()),
            usingFrontLimelight ? (transform.getY()) : (-transform.getY())),
        transform.getRotation());

    double latencyAdjustmentSec = 0.00;

    Pose2d curPose = getPose();
    Pose2d pastPose = getPastBufferedPose(latencyAdjustmentSec);

    final boolean useLatencyAdjustment = false;

    targetPose = useLatencyAdjustment
        ? Optional.of(pastPose.plus(flipTransform))
        : Optional.of(curPose.plus(flipTransform));
  }

  public PathPlannerPath pathToPoint(Pose2d finalPose, double startingSpeedMetersPerSec, double finalSpeedMetersPerSec) {
    Pose2d curPose = getPose();
    Transform2d finalTransform = new Transform2d(finalPose.getTranslation(), finalPose.getRotation());
    System.out.println(
        "Trajectory Transform: " + finalTransform.getX() + " " + finalTransform.getY());

    Rotation2d finalHolonomicRotation = finalPose.getRotation();

    List<Waypoint> bezierTranslations = PathPlannerPath.waypointsFromPoses(curPose, finalPose);
    PathPlannerPath path = new PathPlannerPath(
        bezierTranslations,
        new PathConstraints(
            DriveCal.MEDIUM_LINEAR_SPEED_METERS_PER_SEC,
            DriveCal.MEDIUM_LINEAR_ACCELERATION_METERS_PER_SEC_SQ,
            DriveCal.MEDIUM_ANGULAR_SPEED_RAD_PER_SEC,
            DriveCal.MEDIUM_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ),
        new IdealStartingState(startingSpeedMetersPerSec, curPose.getRotation()),
        new GoalEndState(finalSpeedMetersPerSec, finalHolonomicRotation));

    return path;
  }

  public void driveToPoint(Pose2d point) {
    targetPose = Optional.of(point);
    Optional<PathPlannerPath> path = poseToPath(new Translation2d(lastSetChassisSpeeds.vxMetersPerSecond, lastSetChassisSpeeds.vyMetersPerSecond).getNorm());

    followTrajectoryCommand(path.get(), false);
  }

  public Optional<PathPlannerPath> poseToPath(double startingSpeedMetersPerSec) {
    Pose2d curPose = getPose();

    if (!targetPose.isPresent()) {
      return Optional.empty();
    }

    Pose2d finalPose = targetPose.get();
    List<Waypoint> bezierTranslations = PathPlannerPath.waypointsFromPoses(curPose, finalPose);

    Rotation2d finalHolonomicRotation = Rotation2d.fromDegrees(0.0);

    PathPlannerPath path = new PathPlannerPath(
        bezierTranslations,
        new PathConstraints(
            DriveCal.MEDIUM_LINEAR_SPEED_METERS_PER_SEC,
            DriveCal.MEDIUM_LINEAR_ACCELERATION_METERS_PER_SEC_SQ,
            DriveCal.MEDIUM_ANGULAR_SPEED_RAD_PER_SEC,
            DriveCal.MEDIUM_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ),
        new IdealStartingState(startingSpeedMetersPerSec, curPose.getRotation()),
        new GoalEndState(0.0, finalHolonomicRotation));

    return Optional.of(path);
  }

  public void throttle(double throttleValue) {
    throttleMultiplier = throttleValue;
  }

  public void stopDriving() {
    drive(0, 0, 0, true);
  }

  public Command stopDrivingCommand() {
    return new InstantCommand(this::stopDriving, this);
  }

  public Command turnInPlace(double timeoutSec) {
    return new RunCommand(
            () -> {
              rotateOrKeepHeading(0, 0, 0, true, -1);
            })
        .withTimeout(timeoutSec);
  }

  public void considerZeroingSwerveEncoders() {
    frontLeft.considerZeroingEncoder();
    frontRight.considerZeroingEncoder();
    rearLeft.considerZeroingEncoder();
    rearRight.considerZeroingEncoder();
  }

  public void throttleSpeed(boolean useHalfSpeed) {
    frontLeft.throttleSpeed(useHalfSpeed);
    frontRight.throttleSpeed(useHalfSpeed);
    rearLeft.throttleSpeed(useHalfSpeed);
    rearRight.throttleSpeed(useHalfSpeed);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Throttle multiplier",
        () -> {
          return throttleMultiplier;
        },
        null);
    builder.addDoubleProperty(
        "Target Heading (deg)",
        () -> {
          return targetHeadingDegrees;
        },
        null);
    builder.addDoubleProperty("Gyro Yaw (deg)", () -> gyro.getYaw().getValueAsDouble(), null);
    builder.addDoubleProperty("Odometry X (m)", () -> getPose().getX(), null);
    builder.addDoubleProperty("Odometry Y (m)", () -> getPose().getY(), null);
    builder.addDoubleProperty(
        "Odometry Yaw (deg)", () -> getPose().getRotation().getDegrees(), null);
    builder.addDoubleProperty(
        "Front Left Abs Encoder (rad)", frontLeft::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Front Right Abs Encoder (rad)", frontRight::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Rear Left Abs Encoder (rad)", rearLeft::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Rear Right Abs Encoder (rad)", rearRight::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Front Left Module Pos (rad)", () -> frontLeft.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Right Module Pos (rad)", () -> frontRight.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Rear Left Module Pos (rad)", () -> rearLeft.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Rear Right Module Pos (rad)", () -> rearRight.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Left Module Pos (deg)", () -> frontLeft.getPosition().angle.getDegrees(), null);
    builder.addDoubleProperty(
        "Front Right Module Pos (deg)", () -> frontRight.getPosition().angle.getDegrees(), null);
    builder.addDoubleProperty(
        "Rear Left Module Pos (deg)", () -> rearLeft.getPosition().angle.getDegrees(), null);
    builder.addDoubleProperty(
        "Rear Right Module Pos (deg)", () -> rearRight.getPosition().angle.getDegrees(), null);
    builder.addDoubleProperty(
        "Front Left Distance (m)", () -> frontLeft.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Front Right Distance (m)", () -> frontRight.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Rear Left Distance (m)", () -> rearLeft.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Rear Right Distance (m)", () -> rearRight.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Rear Right Velocity Error (mps)",
        () ->
            rearRight.desiredState.speedMetersPerSecond - rearRight.getState().speedMetersPerSecond,
        null);
    builder.addDoubleProperty("Keep Heading PID [0,1]", () -> KeepHeadingPID, null);
    builder.addDoubleProperty("Keep Heading FF [0,1]", () -> KeepHeadingFF, null);
    builder.addDoubleProperty("Rotation Controller Input", () -> rotControllerInput, null);
    builder.addDoubleProperty(
        "Yaw error",
        () -> (targetHeadingDegrees - getPose().getRotation().getDegrees()) % 360,
        null);
    builder.addDoubleProperty("Target Heading (tag detection)", () -> tagTargetHeading, null);
    builder.addDoubleProperty(
        "Front left desired speed mps", () -> frontLeft.desiredState.speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Front right desired speed mps", () -> frontRight.desiredState.speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Rear left desired speed mps", () -> rearLeft.desiredState.speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Rear right desired speed mps", () -> rearRight.desiredState.speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Front left current speed mps", () -> frontLeft.getState().speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Front right current speed mps", () -> frontRight.getState().speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Rear left current speed mps", () -> rearLeft.getState().speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Rear right current speed mps", () -> rearRight.getState().speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Front left desired position", () -> frontLeft.desiredState.angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front right desired position", () -> frontRight.desiredState.angle.getRadians(), null);
    builder.addDoubleProperty(
        "Rear left desired position", () -> rearLeft.desiredState.angle.getRadians(), null);
    builder.addDoubleProperty(
        "Rear right desired position", () -> rearRight.desiredState.angle.getRadians(), null);
    builder.addDoubleProperty(
        "yaw offset treemap value",
        () ->
            calculateYawOffsetDeg(
                Units.radiansToDegrees(lastSetChassisSpeeds.omegaRadiansPerSecond)),
        null);
  }
}