package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
/*-
 * Contains:
 *      - setLED() : sets LEDs to rainbow colors
 * @author Richie Xue
 */
public class LEDRainbow implements LEDInstruction {
    private int m_rainbowFirstPixelHue = 0;

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        for (int i = 0; i < ledsBuffer.getLength(); i++) {
            final int hue = (m_rainbowFirstPixelHue + (i * 180 / ledsBuffer.getLength())) % 180;
            ledsBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
    }
}