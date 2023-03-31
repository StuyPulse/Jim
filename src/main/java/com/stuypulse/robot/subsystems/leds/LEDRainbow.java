package com.stuypulse.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/*-
 * Contains:
 *      - setLED() : sets LEDs to rainbow colors
 * @author Richie Xue
 * @author Jo Walkup
 */
public class LEDRainbow implements LEDInstruction {

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        int m_rainbowFirstPixelHue = 0;
        for (int i = 0; i < ledsBuffer.getLength(); ++i) {
            // calculate rainbow color
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledsBuffer.getLength())) % 180;

            // set the i-th led to the rainbow color
            ledsBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;   
    }
}