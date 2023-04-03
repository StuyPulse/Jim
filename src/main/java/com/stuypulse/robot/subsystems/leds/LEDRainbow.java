package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.util.LEDColor;

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
        LEDColor[] rainbow = new LEDColor[]{LEDColor.RED, LEDColor.RED_ORANGE, LEDColor.ORANGE, LEDColor.YELLOW, LEDColor.LAWN_GREEN, LEDColor.GREEN, LEDColor.DARK_GREEN, LEDColor.BLUE_GREEN, LEDColor.BLUE, LEDColor.DARK_BLUE, LEDColor.BLUE_VIOLET, LEDColor.VIOLET, LEDColor.PURPLE, LEDColor.PINK};
        System.out.println("RAINBOW SETLED WAS CALLED!");
        LEDController.getInstance().setColor(LEDColor.RED, 0.75);
        // int j = 0;
        // int i = 0;
        // while(i < rainbow.length) {
        //     System.out.println("j (led) = " + j);
        //     System.out.println("i (rainbow) = " + i);
        //     // for (int rgb = 0; rgb < 255; rgb++) {
        //     //     if (rgb < 85) {
        //     //         ledsBuffer.setRGB(i, rgb * 3, 255 - rgb * 3, 0);
        //     //     }
        //     //     else if (rgb < 170) {
        //     //         ledsBuffer.setRGB(i, 255 - (rgb - 85) * 3, 0, (rgb - 85) * 3);
        //     //     }
        //     //     else {
        //     //         ledsBuffer.setRGB(i, 0, (rgb-170) * 3, 255 - (rgb - 170) * 3);
        //     //     }
        //     // }
        //     ledsBuffer.setRGB(j, rainbow[i].getRed(), rainbow[i].getGreen(), rainbow[i].getBlue());
        //     j++;
        //     i = j / 4;
    }
}