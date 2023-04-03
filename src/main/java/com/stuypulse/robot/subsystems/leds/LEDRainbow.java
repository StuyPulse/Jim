package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.util.LEDColor;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
/*-
 * Contains:
 *      - setLED() : sets LEDs to rainbow colors
 * @author Richie Xue
 * @author Jo Walkup
 */
public class LEDRainbow implements LEDInstruction {
    StopWatch timer;
    int colorCounter;
    boolean firstRun;
    
    public LEDRainbow() {
        timer = new StopWatch();
        colorCounter = 0;
        firstRun = true;
    }     

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        System.out.println("RAINBOW SETLED WAS CALLED!");
        LEDColor[] rainbow = new LEDColor[]{LEDColor.RED, LEDColor.RED_ORANGE, LEDColor.ORANGE, LEDColor.YELLOW, LEDColor.LAWN_GREEN, LEDColor.GREEN, LEDColor.DARK_GREEN, LEDColor.BLUE_GREEN, LEDColor.BLUE, LEDColor.DARK_BLUE, LEDColor.BLUE_VIOLET, LEDColor.VIOLET, LEDColor.PURPLE, LEDColor.PINK};
        ledsBuffer.setRGB(0, 255, 255, 255);
        LEDColor color;
        if(firstRun || timer.getTime() > 3 * Math.sin(colorCounter / 2.0)) {
            timer.reset();
            System.out.println("changed something");
            firstRun = false;
            for(int i = 0; i < ledsBuffer.getLength(); i++) {
                //color = rainbow[(i + colorCounter) % rainbow.length];
                //ledsBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
                ledsBuffer.setHSV(i, colorCounter, 255, 128);
                colorCounter += 5;
                if(colorCounter == 180) colorCounter = 0;
            }
        }
    }
}