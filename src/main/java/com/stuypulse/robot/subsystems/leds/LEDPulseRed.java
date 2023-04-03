package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDPulseRed implements LEDInstruction {
    public StopWatch stopwatch;

    public LEDPulseRed() {
        stopwatch = new StopWatch();
    }

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        if (stopwatch.getTime() < 0.5) {
            for (int i = 0; i < ledsBuffer.getLength(); i++) {
                ledsBuffer.setRGB(i, 255, 0, 0);
            }
        }
        else {
            stopwatch.reset();
            for (int i = 0; i < ledsBuffer.getLength(); i++) {
                ledsBuffer.setRGB(i, 0, 0, 0);
            }
        }
    }
}

