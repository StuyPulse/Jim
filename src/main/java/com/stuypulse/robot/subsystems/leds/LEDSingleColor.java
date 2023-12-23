package com.stuypulse.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import com.stuypulse.robot.util.SLColor;

public class LEDSingleColor implements LEDInstruction {
    SLColor color;

    public LEDSingleColor(SLColor color) {
        this.color = color;
    }

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        for (int i = 0; i < ledsBuffer.getLength(); i++) {
            ledsBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        }
    }
}
