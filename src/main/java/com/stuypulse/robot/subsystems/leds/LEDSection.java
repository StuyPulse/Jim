package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.util.SLColor;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSection implements LEDInstruction {

    public SLColor[] sections;

    public LEDSection(SLColor[] sections) {
        this.sections = sections;
    }
        

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        int sectionLength = ledsBuffer.getLength() / sections.length;
        
        for (int i = 0; i < sections.length; i++) {
            for (int j = 0; j < sectionLength; j++) {
                ledsBuffer.setRGB(i * sectionLength + j, sections[i].getRed(), sections[i].getGreen(), sections[i].getBlue());
            }
        }
        
    }
    
}
