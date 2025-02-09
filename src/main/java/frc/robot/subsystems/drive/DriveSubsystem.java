package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
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
import edu.wpi.first.math.system.plant.DCMotor;
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

public class DriveSubsystem extends SubsystemBase {

  private double targetHeadingDegrees;

  double tagTargetHeading = 0.0;  

  // Create SwerveModules
  public final SwerveModule frontLeft =
      new SwerveModule(
          RobotMap.FRONT_LEFT_DRIVING_CAN_ID,
          RobotMap.FRONT_LEFT_TURNING_CAN_ID,
          DriveCal.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule frontRight =
      new SwerveModule(
          RobotMap.FRONT_RIGHT_DRIVING_CAN_ID,
          RobotMap.FRONT_RIGHT_TURNING_CAN_ID,
          DriveCal.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule rearLeft =
      new SwerveModule(
          RobotMap.REAR_LEFT_DRIVING_CAN_ID,
          RobotMap.REAR_LEFT_TURNING_CAN_ID,
          DriveCal.BACK_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule rearRight =
      new SwerveModule(
          RobotMap.REAR_RIGHT_DRIVING_CAN_ID,
          RobotMap.REAR_RIGHT_TURNING_CAN_ID,
          DriveCal.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  private final Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON_CAN_ID);
  private ChassisSpeeds lastSetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  public Optional<Pose2d> targetPose = Optional.empty();

  public PoseBuffer poseBuffer = new PoseBuffer();

  double KeepHeadingPID = 0.0;
  double KeepHeadingFF = 0.0;

  SwerveDriveOdometry odometry =
  new SwerveDriveOdometry(
      DriveConstants.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(Constants.PLACEHOLDER_DOUBLE),
      getModulePositions());

  public double throttleMultiplier = 1.0;

  private double rotControllerInput = 0.0;

  public MatchStateUtil matchState;

  private InterpolatingDoubleTreeMap yawOffsetMap;
 
  private InterpolatingDoubleTreeMap velocityToMultiplierMap;

  public DriveSubsystem(MatchStateUtil matchState) {
    intializeGyro();
    this.matchState = matchState;
    
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getCurrentChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setOutputRobotRelativeSpeeds, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new PPHolonomicDriveController(DriveCal.PATH_TRANSLATION_CONTROLLER, DriveCal.PATH_ROTATION_CONTROLLER),
        
        new RobotConfig(DriveConstants.ROBOT_MASS_KG, DriveConstants.ROBOT_MOMENT_OF_INERTIA, new ModuleConfig(DriveConstants.WHEEL_DIAMETER_METERS, DriveConstants.MAX_SPEED_METERS_PER_SECOND, DriveConstants.WHEEL_COEFFICIENT_OF_FRICTION, null, DriveConstants.MOTOR_CURRENT_LIMIT, 4), DriveConstants.FRONT_LEFT_MODULE_OFFSET, DriveConstants.FRONT_RIGHT_MODULE_OFFSET, DriveConstants.REAR_LEFT_MODULE_OFFSET, DriveConstants.REAR_RIGHT_MODULE_OFFSET),

        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },

        this
        );

    yawOffsetMap = new InterpolatingDoubleTreeMap();
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

    // Reset position to zero, this may be overwritten by a path at the start of auto
    gyro.reset();
    Timer.delay(0.1);
    gyro.reset();
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
    ChassisSpeeds chassisSpeeds =
        DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState());
    return chassisSpeeds;
  }

  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    // TODO test arbitrarily making this larger to see if it helps
    Pose2d futureRobotPose =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds =
        new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S);
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
    var swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);
    setModuleStates(swerveModuleStates);
  }
}