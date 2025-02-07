package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import frc.robot.RobotMap;
import java.util.TreeMap;

public class Lights {
  /**
   * Tree map of LightCode enums and integers that represent the R, G, B values of each light code
   * enum
   */
  private TreeMap<LightCode, Integer[]> lightOptionsMap;

  private CANdle candle = new CANdle(RobotMap.CANDLE_CAN_ID);
  private CANdleConfiguration config = new CANdleConfiguration();
  private LightCode currentLightStatus = LightCode.OFF;

  public enum LightCode {
    OFF, // Black
    DISABLED, // ORANGE
    READY_TO_INTAKE, // Blink RED
    HAS_CORAL, // GREEN
    SCORE_PREP, // BLUE
    READY_TO_SCORE, // RAINBOW
    CLIMB_PREP_SHALLOW, // PURPLE
    CLIMB_PREP_DEEP, // BLINK PURPLE
    READY_TO_CLIMB, // RAINBOW
    CLIMBING, // BLINK BLUE
    PARTY_MODE, // RAINBOW ANIMATION
    HOME // red
  }

  public Lights() {
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;
    candle.configAllSettings(config);

    lightOptionsMap = new TreeMap<LightCode, Integer[]>();
    lightOptionsMap.put(LightCode.OFF, new Integer[] {0, 0, 0});
    lightOptionsMap.put(LightCode.DISABLED, new Integer[] {255, 255, 0});
    lightOptionsMap.put(LightCode.READY_TO_INTAKE, new Integer[] {255, 0, 0}); // bink red
    lightOptionsMap.put(LightCode.HAS_CORAL, new Integer[] {0, 255, 0});
    lightOptionsMap.put(LightCode.SCORE_PREP, new Integer[] {0, 0, 255});
    lightOptionsMap.put(LightCode.CLIMB_PREP_SHALLOW, new Integer[] {255, 0, 255});
    lightOptionsMap.put(LightCode.CLIMB_PREP_DEEP, new Integer[] {255, 0, 255}); // blink purple
    lightOptionsMap.put(LightCode.CLIMBING, new Integer[] {0, 0, 255}); // blink blue
    lightOptionsMap.put(LightCode.HOME, new Integer[] {255, 0, 0}); // redk
  }

  public void setLEDColor(LightCode light) {
    currentLightStatus = light;
    setLEDs();
  }

  private void setLEDs() {
    if (currentLightStatus == LightCode.PARTY_MODE
        || currentLightStatus == LightCode.READY_TO_CLIMB
        || currentLightStatus == LightCode.READY_TO_SCORE) {
      setRainbow();
    } else if (currentLightStatus == LightCode.CLIMBING
        || currentLightStatus == LightCode.CLIMB_PREP_DEEP
        || currentLightStatus == LightCode.READY_TO_INTAKE) {
      setBlink(currentLightStatus);
    } else {
      candle.animate(null);
      candle.setLEDs(
          lightOptionsMap.get(currentLightStatus)[0],
          lightOptionsMap.get(currentLightStatus)[1],
          lightOptionsMap.get(currentLightStatus)[2]);
    }
  }

  public void setBlink(LightCode light) {
    currentLightStatus = light;

    var twinkle =
        new StrobeAnimation(
            lightOptionsMap.get(currentLightStatus)[0],
            lightOptionsMap.get(currentLightStatus)[1],
            lightOptionsMap.get(currentLightStatus)[2],
            0,
            0.1,
            -1,
            0);
    // twinkle.setDivider(TwinklePercent.Percent42);
    candle.animate(twinkle);
  }

  private void setRainbow() {
    RainbowAnimation rainbowAnim =
        new RainbowAnimation(
            LightsConstants.LIGHT_BRIGHTNESS,
            LightsConstants.LIGHT_SPEED,
            LightsConstants.NUM_CANDLE_LEDS);
    candle.animate(rainbowAnim);
  }
}
