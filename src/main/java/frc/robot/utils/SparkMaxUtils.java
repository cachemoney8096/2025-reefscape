package frc.robot.utils;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;

import java.util.function.BooleanSupplier;

/** Mostly borrowed from 3005 */
public class SparkMaxUtils {

  /**
   * @param error API return value
   * @return
   */
  public static int check(REVLibError error) { // TODO: Add proper support for the new config system.
    return error == REVLibError.kOk ? 0 : 1;
  }

  public static class UnitConversions {

    /** Sets the encoder to read degrees after some gear ratio. */
    public static REVLibError setDegreesFromGearRatio(SparkMax sparkMax, double ratio) {
      double degreesPerRotation = 360.0 / ratio;

      SparkMaxConfig config = new SparkMaxConfig();

      config.encoder.positionConversionFactor(degreesPerRotation);
      config.encoder.velocityConversionFactor(degreesPerRotation);

      return sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Sets the encoder to read radians after some gear ratio. */
    public static REVLibError setRadsFromGearRatio(SparkMax sparkMax, double ratio) {
      double radsPerRotation = (2.0 * Math.PI) / ratio;

      SparkMaxConfig config = new SparkMaxConfig();

      config.encoder.positionConversionFactor(radsPerRotation);

      return sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the encoder to read a linear distance with a gear ratio and a conversion to linear.
     *
     * @param sparkMaxEncoder The encoder to set on.
     * @param ratio The gear ratio from motor to the winch (or other rotary-to-linear converter). >1
     *     is a reduction.
     * @param diameter The diameter of the winch (or other rotary-to-linear converter).
     */
    public static REVLibError setLinearFromGearRatio(SparkMax sparkMax, final double ratio, final double diameter) {
      double linearDistPerRotation = diameter * Math.PI / ratio;

      SparkMaxConfig config = new SparkMaxConfig();
      
      config.encoder.positionConversionFactor(linearDistPerRotation);
      config.encoder.velocityConversionFactor(linearDistPerRotation);

      return sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  /**
   * Takes a function returning true on success, and tries running it until it succeeds up to the
   * max retry attempts
   */
  public static void initWithRetry(BooleanSupplier initFunction, int maxRetryAttempts) {
    int numAttempts = 0;
    while (!initFunction.getAsBoolean()) {
      numAttempts++;
      if (numAttempts > maxRetryAttempts) {
        break;
      }
    }
  }
}