package frc.robot.subsystems.lights;
import frc.robot.Constants;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import frc.robot.RobotMap;
import java.util.TreeMap;

public class Lights {
  /**
   * Tree map of light code enums and integers that represent the R, G, B values of each light code
   * enum
   */
  private TreeMap<LightCode, Integer[]> lightOptionsMap;
  
  private CANdle candle = new CANdle(RobotMap.CANDLE_CAN_ID);
  private CANdleConfiguration config = new CANdleConfiguration();
  private LightCode currentLightStatus = LightCode.OFF;
 
  public enum LightCode {
    READY_TO_INTAKE, // ORANGE
    OFF, // BLACK
    HAS_CORAL, // GREEN
    SCORE_PREP, // BLUE
    READY_TO_SCORE, // YELLOW
    CLIMB_PREP_SHALLOW, // WHITE
    CLIMB_PREP_DEEP, // RED
    READY_TO_CLIMB, // PURPLE
    PARTYMODE,
    CLIMBING,
    DISABLED,   
  }

  public Lights() {
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;
    candle.configAllSettings(config);

    lightOptionsMap = new TreeMap<LightCode, Integer[]>();
    lightOptionsMap.put(LightCode.READY_TO_INTAKE, new Integer[] {255, 128, 0});
    lightOptionsMap.put(LightCode.OFF, new Integer[] {0, 0, 0});
    lightOptionsMap.put(LightCode.HAS_CORAL, new Integer[] {128, 255, 0});
    lightOptionsMap.put(LightCode.SCORE_PREP, new Integer[] {0, 0, 255});
    lightOptionsMap.put(LightCode.READY_TO_SCORE, new Integer[] {255, 255, 0});
    lightOptionsMap.put(LightCode.CLIMB_PREP_SHALLOW, new Integer[] {255, 255, 255});
    lightOptionsMap.put(LightCode.CLIMB_PREP_DEEP, new Integer[] {255, 0, 0});
    lightOptionsMap.put(LightCode.READY_TO_CLIMB, new Integer[] {127, 0, 255});
    lightOptionsMap.put(LightCode.DISABLED, new Integer[] {0, 128, 128});
    lightOptionsMap.put(LightCode.CLIMBING, new Integer[] {128, 0, 128});
    lightOptionsMap.put(LightCode.PARTYMODE, new Integer[] {0, 0, 0});
    
  }
 

  public void setLEDColor(LightCode light)
   {
    currentLightStatus = light;
    setLEDs();
  }

  private void setLEDs() 
  {
    if (currentLightStatus == LightCode.PARTYMODE) {
      setRainbow(); 
   
  
      candle.animate(null);
      candle.setLEDs(
      lightOptionsMap.get(currentLightStatus)[0],
      lightOptionsMap.get(currentLightStatus)[1],
      lightOptionsMap.get(currentLightStatus)[2]);}
    }
  

  public void setBlink(LightCode light)
  {
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
// If you can see this you will be cursed by the skibidi demons for all of eternity
// The demons of ancient chinese lore have crucified your soul for eternity
// LEAVE A POSITIVE PR TO UNDO THIS EFFECT!!
// ps. the skibidi demons are watching...