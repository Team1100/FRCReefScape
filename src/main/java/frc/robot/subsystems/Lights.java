// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.LightsConstants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.SubsystemBase;

public class Lights extends SubsystemBase {
  public static Lights m_lights;

  private AddressableLED m_LED;
  private AddressableLEDBuffer m_LEDBuffer;

  private int m_rainbowFirstPixelHue;
  private int m_firstPixelValue;
  private boolean m_blinkState;
  private int[] m_flashIndexQueue;
  private double[] m_flashTimerQueue;
  private double[] m_flashStartQueue;
  private Timer m_timer;

  private Random m_random;

  class LightRange {
    int start;
    int end;

    public LightRange(int start, int end) {this.start = start; this.end = end;}
  }

  public enum LightSection {
    ALL,
    ACTIVE,
    STATUS
  };

  /** Creates a new lights. */
  public Lights() {
    super("Lights");

    m_LED = new AddressableLED(RobotMap.L_LEDS);

    m_LEDBuffer = new AddressableLEDBuffer(Constants.LightsConstants.LED_LENGTH);
    m_LED.setLength(Constants.LightsConstants.LED_LENGTH);

    setData();
    m_LED.start();

    m_rainbowFirstPixelHue = 0;
    m_firstPixelValue = 0;
    m_blinkState = true;

    m_timer = new Timer();
    m_timer.start();
    m_timer.reset();
    
    m_random = new Random();
    m_flashIndexQueue = new int[LightsConstants.kNumFlashes];
    m_flashTimerQueue = new double[LightsConstants.kNumFlashes];
    m_flashStartQueue = new double[LightsConstants.kNumFlashes];
    for (int i = 0; i < LightsConstants.kNumFlashes; i++) {
      m_flashIndexQueue[i] = m_random.nextInt(LightsConstants.LED_LENGTH);
      m_flashTimerQueue[i] = m_timer.get() + m_random.nextDouble(
        LightsConstants.kFlashDelayMinimum,
        LightsConstants.kFlashDelayMaximum
      );
      m_flashStartQueue[i] = m_timer.get();
    }
  }

  public static Lights getInstance() {
    if (m_lights == null) {
      m_lights = new Lights();
    }
    return m_lights;
  }

  public LightRange getLightRange(LightSection section) {
    switch (section) {
      case ACTIVE: {
        return new LightRange(
          LightsConstants.kInversePolarity ? 0 : LightsConstants.kPositionSplitIndex,
          LightsConstants.kInversePolarity ? LightsConstants.LED_LENGTH-LightsConstants.kPositionSplitIndex : LightsConstants.LED_LENGTH
        );
      }
      case STATUS: {
        return new LightRange(
          LightsConstants.kInversePolarity ? LightsConstants.LED_LENGTH-LightsConstants.kPositionSplitIndex : 0,
          LightsConstants.kInversePolarity ? LightsConstants.LED_LENGTH : LightsConstants.kPositionSplitIndex
        );
      }
      case ALL:
      default: {
        return new LightRange(0,Constants.LightsConstants.LED_LENGTH);
      }
    }
  }

  public void setData() {
    m_LED.setData(m_LEDBuffer);
  }

  public void enableLights(int hue, LightSection section) {
    LightRange range = getLightRange(section);
    for (var i = range.start; i < range.end; i++) {
      m_LEDBuffer.setHSV(i, hue, 255, 122);
    }

    setData();
  }

  public void makeRainbow(LightSection section) {
    // Reassign blinks that are past their timer
    for (int i = 0; i < Constants.LightsConstants.kNumFlashes; i++) {
      if (m_timer.get() > m_flashTimerQueue[i]) {
        m_flashIndexQueue[i] = m_random.nextInt(Constants.LightsConstants.LED_LENGTH);
        m_flashTimerQueue[i] = m_timer.get() + m_random.nextDouble(
          Constants.LightsConstants.kFlashDelayMinimum,
          Constants.LightsConstants.kFlashDelayMaximum
        );
        m_flashStartQueue[i] = m_timer.get();
      }
    }
    // For every pixel
    LightRange range = getLightRange(section);
    for (var i = range.start; i < range.end; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to process
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / Constants.LightsConstants.LED_LENGTH)) % 180;
      // Set the value
      boolean isFlash = false;
      double start = 0.0;
      double end = 0.0;
      for (int j = 0; j < Constants.LightsConstants.kNumFlashes; j++) {
        if (i == m_flashIndexQueue[j]) {
          isFlash = true;
          start = m_flashStartQueue[j];
          end = m_flashTimerQueue[j];
        }
      }
      if (isFlash) {
        Color color = Color.fromHSV(hue, 255, 128);
        Color interp = Color.lerpRGB(
          color,
          color,
          1.0-Math.abs(2.0*((m_timer.get()-start)/(end-start)-0.5))
        );
        m_LEDBuffer.setRGB(i, (int)(interp.red*255), (int)(interp.green*255), (int)(interp.blue*255));
      } else m_LEDBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void cool(LightSection section) {
    LightRange range = getLightRange(section);
    for (var i = range.start; i < range.end; i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 130 / Constants.LightsConstants.LED_LENGTH)) % 180;
      m_LEDBuffer.setHSV(i, hue, 255, 128);
    }
    m_rainbowFirstPixelHue += 2;
    m_rainbowFirstPixelHue %= 90;
  }

  public void warm(LightSection section) {
    // For every pixel
    LightRange range = getLightRange(section);
    for (var i = range.start; i < range.end; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = 170 + (m_rainbowFirstPixelHue + (i * 120 / m_LEDBuffer.getLength())) % 40;
      // Set the value
      m_LEDBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 2;
    // Check bounds
    m_rainbowFirstPixelHue %= 60;
  }

  public void moveLights(int hue, LightSection section) {
    LightRange range = getLightRange(section);
    for (var i = range.start; i < range.end; i++) {
      final var value = (m_firstPixelValue + (i * 255 / Constants.LightsConstants.LED_LENGTH)) % 255;
      m_LEDBuffer.setHSV(i, hue, 255, value);
    }
    
    // what "moves" the program
    m_firstPixelValue += 10;
    m_firstPixelValue %= 255;
  }

  public void blinkLights(int hue, LightSection section) {
    LightRange range = getLightRange(section);
    for (var i = range.start; i < range.end; i++) {
      m_LEDBuffer.setHSV(i, hue, 255, m_blinkState ? 255 : 0);
    }
    if (m_timer.hasElapsed(Constants.LightsConstants.kBlinkDelay)) {
      m_blinkState = !m_blinkState;
      m_timer.reset();
    }
  }
  public void disableLights(LightSection section) {
    LightRange range = getLightRange(section);
    for (var i = range.start; i < range.end; i++) {
      m_LEDBuffer.setLED(i, Color.kBlack);
    }

    setData();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
