package frc.robot.subsystems.ScoringLimelight;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;
import java.util.List;
import java.util.Optional;

/** Limelight for identifying April Tags @ Human Player Station Code adapted from Team #3005 */
public class ScoringLimelight extends SubsystemBase {
  private final double kCameraPitchAngleDegrees;
  private final double kCameraHeight;
  private final double kTargetHeight;
  private final double kImageCaptureLatency = 11.0; // TODO from last year

  // Simulation functions
  private SimDevice m_simDevice;
  private SimDouble m_targetArea;
  private SimDouble m_skew;
  private SimDouble m_latency;
  private SimDouble m_tx;
  private SimDouble m_ty;
  private SimBoolean m_valid;

  // NT published variables when using translation api
  private double m_lastDistance = 0.0;
  private double m_lastX = 0.0;
  private double m_lastY = 0.0;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-scoring");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tclass = table.getEntry("tclass");
  NetworkTableEntry thor = table.getEntry("thor");
  NetworkTableEntry tvert = table.getEntry("tvert");
  NetworkTableEntry tshort = table.getEntry("tshort");
  NetworkTableEntry tlong = table.getEntry("tlong");

  // AprilTag detection
  private Optional<Transform2d> robotToScoringLocation = Optional.empty();

  /**
   * @param pitchAngleDegrees pitch angle from normal in degrees. Looking straight out is 0, and
   *     increasing as the camera is tilted towards the ceiling.
   * @param heightMeters height of the camera measured from the lens to the ground in meters.
   * @param targetHeightMeters height to the center of the target in meters.
   */
  public ScoringLimelight(
      double pitchAngleDegrees, double heightMeters, double targetHeightMeters) {
    this.kCameraPitchAngleDegrees = pitchAngleDegrees;
    this.kCameraHeight = heightMeters;
    this.kTargetHeight = targetHeightMeters;
    setLimelightValues(
        Constants.limelightLedMode.OFF,
        Constants.limelightCamMode.VISION_PROCESSING,
        Constants.limelightPipeline.TAG_PIPELINE);

    m_simDevice = SimDevice.create("limelight-scoring");
    if (m_simDevice != null) {
      m_targetArea = m_simDevice.createDouble("Target Area", Direction.kBidir, 0.0);
      m_skew = m_simDevice.createDouble("Skew", Direction.kBidir, 0.0);
      m_latency = m_simDevice.createDouble("Latency", Direction.kBidir, 0.0);
      m_tx = m_simDevice.createDouble("Tx", Direction.kBidir, 0.0);
      m_ty = m_simDevice.createDouble("Ty", Direction.kBidir, 0.0);
      m_valid = m_simDevice.createBoolean("Valid", Direction.kBidir, false);
    }
  }

  /**
   * @param botPoseTargetSpace is the robot's 3D pose in the target's space (where the target is the
   *     origin)
   * @return Transform2d with translation and rotation with the robot's pose relative to the target
   */
  private static Transform2d getBotFromTarget(Pose3d botPoseTargetSpace) {
    /**
     * Target space: 3d Cartesian Coordinate System with (0,0,0) at the center of the target.
     *
     * <p>X is positive when the target is to the right.
     *
     * <p>Y is positive when pointing down.
     *
     * <p>Z is positive when pointing out of the target (orthogonal to target’s plane, towards us).
     */

    /**
     * We convert to 2d target space: X is positive pointing out of the target (Z is now X). Y is
     * positive when the target is to the right (X is now Y) Positive yaw is based on the positive Z
     * axis pointing up
     */
    Translation2d translation =
        new Translation2d(botPoseTargetSpace.getZ(), botPoseTargetSpace.getX());
    Rotation2d rot =
        Rotation2d.fromDegrees(-botPoseTargetSpace.getRotation().getY()); // TODO why pitch?

    System.out.println("Tag at " + -botPoseTargetSpace.getRotation().getY() + " deg");
    return new Transform2d(translation, rot);
  }

  public static boolean validScoringTags(double tagId) {
    long tagIdRounded = Math.round(tagId);
    List<Integer> humanPlayerStationTags = List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    return humanPlayerStationTags.contains((int) tagIdRounded);
  } // TODO if something breaks, could be this

  /**
   * @param targets array of all targets the LimeLight sees
   * @return array index of the tag closest to the robot
   */
  public static int chooseTag(LimelightTarget_Fiducial[] targets) {
    int numTags = targets.length;
    double minDistMeters = Double.MAX_VALUE;
    int bestTag = 0;
    for (int tagIndex = 0; tagIndex < numTags; tagIndex++) {
      LimelightTarget_Fiducial target = targets[tagIndex];
      if (!validScoringTags(target.fiducialID)) {
        continue;
      }
      double targetDistance = target.getTargetPose_RobotSpace().getTranslation().getNorm();
      if (targetDistance < minDistMeters) {
        minDistMeters = targetDistance;
        bestTag = tagIndex;
      }
    }
    return bestTag;
  }

  /**
   * @return Optional<Transform2d> of how to get to the tag (accounting for offset from HPS)
   */
  public Optional<Transform2d> checkForTag() {
    // if 0 (no target found) or -1 (m_simDevice is null)
    if (getValidTarget() != 1) {
      return Optional.empty();
    }

    Pose3d cameraToTag =
        LimelightHelpers.getTargetPoseCameraSpace(ScoringLimelightConstants.SCORING_LIMELIGHT_NAME);
    Transform2d robotToTag =
        new Transform2d(
            new Translation2d(cameraToTag.getZ(), -cameraToTag.getX()),
            Rotation2d.fromRadians(-cameraToTag.getRotation().getY()));

    System.out.println("robotToTag: " + robotToTag);
    System.out.println("trap offset: " + ScoringLimelightCal.REEF_OFFSET);
    System.out.println("sum: " + robotToTag.plus(ScoringLimelightCal.REEF_OFFSET));

    return Optional.of(robotToTag.plus(ScoringLimelightCal.REEF_OFFSET));
  }

  public double getLatencySeconds() {
    return (LimelightHelpers.getLatency_Capture(ScoringLimelightConstants.SCORING_LIMELIGHT_NAME)
            + LimelightHelpers.getLatency_Pipeline(
                ScoringLimelightConstants.SCORING_LIMELIGHT_NAME))
        / 1000.0;
  }

  /**
   * @param mode LED operating mode.
   */
  public void setLedMode(Constants.limelightLedMode mode) {
    table.getEntry("ledMode").setNumber(mode.ordinal());
  }

  /**
   * @param mode Camera operating mode.
   */
  public void setCamMode(Constants.limelightCamMode mode) {
    table.getEntry("camMode").setNumber(mode.ordinal());
  }

  /**
   * @param line Pipeline index
   */
  public void setPipeline(Constants.limelightPipeline line) {
    table.getEntry("pipeline").setNumber(line.ordinal());
  }

  /**
   * Gets the current vision pipeline
   *
   * @return pipeline
   */
  public Constants.limelightPipeline getPipeline() {
    return Constants.limelightPipeline.values()[(int) table.getEntry("getpipe").getDouble(0)];
  }

  public void setLimelightValues(
      Constants.limelightLedMode ledMode,
      Constants.limelightCamMode camMode,
      Constants.limelightPipeline line) {
    setLedMode(ledMode);
    setCamMode(camMode);
    setPipeline(line);

    SmartDashboard.putString("LED Mode", ledMode.name());
    SmartDashboard.putString("Cam Mode", camMode.name());
  }

  /**
   * @return validObject is 1 when LimeLight sees target. 0 when LimeLight doesn't see target. -1
   *     when LimeLight isn't working.
   */
  public double getValidTarget() {
    if (m_simDevice != null) {
      return m_valid.get() ? 1 : 0;
    }
    return table.getEntry("tv").getDouble(-1);
  }

  /**
   * @return xOffSet - horizontal offset (left to right) from LimeLight crosshair to target. (LL2:
   *     -29.8 to 29.8 degrees)
   */
  public double getOffSetX() {
    if (m_simDevice != null) {
      return m_tx.get();
    }
    return table.getEntry("tx").getDouble(0.0);
  }

  /**
   * @return yOffSett - vertical offset from LimeLight crosshair to target (LL1: -20.5 degrees to
   *     20.5 degrees | LL2: -24.85 to 24.85 degrees)
   */
  public double getOffSetY() {
    if (m_simDevice != null) {
      return m_ty.get();
    }
    return table.getEntry("ty").getDouble(0.0);
  }

  /**
   * @return targetArea - target Area (0% to 100% of the image)
   */
  public double getTargetArea() {
    if (m_simDevice != null) {
      return m_targetArea.get();
    }
    return table.getEntry("ta").getDouble(0.0);
  }

  /**
   * @return skew - skew or rotation (-90 degrees to 0 degrees)
   */
  public double getSkew() {
    if (m_simDevice != null) {
      return m_skew.get();
    }
    return table.getEntry("ts").getDouble(0.0);
  }

  /**
   * @return latency - The pipeline’s latency contribution in seconds. Add at least the image
   *     capture latency (kImageCaptureLatency) defined above.
   */
  public double getLatency() {
    if (m_simDevice != null) {
      return m_latency.get() + kImageCaptureLatency;
    }
    return (table.getEntry("tl").getDouble(0.0) + kImageCaptureLatency) / 1e3;
  }

  /**
   * Get the timestamp of the last update to the network table. This can be used to get a better
   * estimate of the total latency. That is (lastUpdate - latency).
   *
   * @return timestamp of the last update to the latency update in seconds.
   */
  public double getLastTimestamp() {
    if (m_simDevice != null) {
      return Timer.getFPGATimestamp();
    }
    return table.getEntry("tl").getLastChange() / 1e6;
  }

  /**
   * @return true if the LimeLight has found a target, otherwise false
   */
  public boolean isValidTarget() {
    return getValidTarget() > 0;
  }

  /**
   * @return true if the LimeLight does not have value of -1, otherwise false
   */
  public boolean checkConnection() {
    return getValidTarget() != -1.0;
  }

  /**
   * Get a 2d translation from the camera to the target, including normalization to handle the
   * effects of angle to target. See the below discussion.
   * https://www.chiefdelphi.com/t/what-does-limelight-skew-actually-measure/381167/7
   *
   * @param targetHeightMeters use a target height other than what is provided in constructor.
   * @return Translation2d from the camera to the target (camera relative, where X is out of the
   *     camera and positive Y is to the left of the camera's POV.
   */
  public Translation2d getTargetTranslation(double targetHeightMeters) {
    /**
     * This function uses the limelight coordinates until the end of the function. That is, the x
     * axis is left/right, horizontal to the field. The y axis is 'up/down' normal to the field
     * surface, and z is away from the camera, horizontal to the field surface.
     *
     * <p>The output coordinates are only a translation in x/y to the target, where x is out of the
     * camera, horizontal to the field, and y is positive to the left from the perspective of the
     * camera.
     */
    double diff_height = targetHeightMeters - kCameraHeight;

    // x is the "left/right" to the target from the camera
    double x = Math.tan(Math.toRadians(getOffSetX()));

    // y is the 'up/down' to the target from the center of the camera
    double y = Math.tan(Math.toRadians(getOffSetY()));

    // z is straight out of the camera
    double z = 1.0;
    double length =
        Math.sqrt(x * x + y * y + z * z); // 3D distance formula (compared to the origin/camera)

    // Normalized vector components
    double x_norm = x / length;
    double y_norm = y / length;
    double z_norm = z / length;

    // Rotate vector by camera degrees around camera x axis
    Translation2d zy_translation =
        new Translation2d(z_norm, y_norm)
            .rotateBy(Rotation2d.fromDegrees(kCameraPitchAngleDegrees));
    z_norm = zy_translation.getX();
    y_norm = zy_translation.getY();

    // prevent divide by zero
    if (Math.abs(y_norm) < 1e-4) {
      y_norm = Math.signum(y_norm) * 1e-4;
    }

    /**
     * Find the intersection between the target in space, and the vector pointing to the target
     *
     * <p>This becomes a vector [x_norm, y_norm, z_norm] * d = [x_target, diff_height, z_target]
     *
     * <p>x_norm * d = x_target y_norm * d = diff_height z_target * d = z_target
     */
    double scaling = diff_height / y_norm;
    double x_target = scaling * x_norm;
    double z_target = scaling * z_norm;
    double distance = Math.hypot(z_target, x_target);

    // Convert camera coordinates to our conventions
    Translation2d result = new Translation2d(distance, new Rotation2d(z_target, -x_target));

    // For NT so this function doesn't need to be called multiple times
    m_lastDistance = distance;
    m_lastX = result.getX();
    m_lastY = result.getY();

    return result;
  }

  /**
   * Get a 2d translation from the camera to the target, including normalization to handle the
   * effects of angle to target. See the below discussion.
   * https://www.chiefdelphi.com/t/what-does-limelight-skew-actually-measure/381167/7
   *
   * @return Translation2d from the camera to the target. This is camera relative, x is out of the
   *     camera and positive y is to the left from the camera's point of view.
   */
  public Translation2d getTargetTranslation() {
    return getTargetTranslation(kTargetHeight);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Latency", () -> getLatency(), null);
    builder.addDoubleProperty("Tx", () -> getOffSetX(), null);
    builder.addDoubleProperty("Ty", () -> getOffSetY(), null);
    builder.addBooleanProperty("Valid Target", () -> isValidTarget(), null);
    builder.addBooleanProperty("Connected", () -> checkConnection(), null);
  }
}
