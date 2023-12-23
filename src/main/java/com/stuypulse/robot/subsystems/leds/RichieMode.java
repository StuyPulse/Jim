package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.util.SLColor;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * LED Color that travels through the strip
 *
 * @author Colyi Chen
 * @author Naowal Rahman
 * @author Mustafa Abdullah
 * @author Kalimul Kaif
 * @author Rahel Arka
 */

public class RichieMode implements LEDInstruction {
    public SLColor color;
    private StopWatch stopwatch;
    private int index;
    private int prevIndex;

    public RichieMode(SLColor color) {
        this.color = color;
        stopwatch = new StopWatch();
    }

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        if(stopwatch.getTime() > 0.25) {
            ledsBuffer.setRGB(index, color.getRed(), color.getGreen(), color.getBlue());
            ledsBuffer.setRGB(prevIndex, 0, 0, 0);
            prevIndex = index;
            ++index;

            if(index == ledsBuffer.getLength()) {
                index = 0;
            }

            stopwatch.reset();
        }
        
    }
    
}
