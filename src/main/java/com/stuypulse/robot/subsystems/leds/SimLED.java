package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.stuylib.util.StopWatch;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class SimLED extends LEDController{
    private static SimLED simulation;

    static {
        simulation = new SimLED();
    }

    //@Override
    public static SimLED getInstance() {
        return simulation;
    }

    // Motor that controlls the LEDs
    private MechanismRoot2d leds;
    private MechanismRoot2d ledsBuffer;

    // Stopwatch to check when to start overriding manual updates
    private StopWatch lastUpdate;
    private double manualTime;

    // The current color to set the LEDs to
    private LEDColor manualColor;

    protected SimLED() {
        // leds = new MechanismRoot2d("leds", Ports.LEDController.PORT, Ports.LEDController.PORT);
        // ledsBuffer = new MechanismRoot2d("ledsBuffer", Settings.LED.LED_LENGTH, Settings.LED.LED_LENGTH); // get length of led strip ?

        // // set data
        // leds.setLength(ledsBuffer.getLength());
        // leds.setData(ledsBuffer);
        // leds.start();

        this.lastUpdate = new StopWatch();
    }

    public void setColor(LEDColor color, double time) {
        manualColor = color;
        manualTime = time;
        lastUpdate.reset();
    }

    private void forceSetLEDs(LEDColor color) {
        // for (int i = 0; i < ledsBuffer.getLength(); i++) {
        //     ledsBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        // }
        // leds.setData(ledsBuffer);
    }

    public void setLEDConditions() {
    }

    public LEDColor getDefaultColor() {
        switch (Manager.getInstance().getGamePiece()) {
            case CUBE: return LEDColor.PURPLE;
            case CONE_TIP_IN: return LEDColor.YELLOW;
            case CONE_TIP_UP: return LEDColor.GREEN;
            case CONE_TIP_OUT: return LEDColor.ORANGE;
            default: return LEDColor.OFF;
        }
    }

    @Override
    public void periodic() {
        // If we called .setColor() recently, use that value
        if (Robot.getMatchState() == MatchState.AUTO || lastUpdate.getTime() < manualTime) {
            forceSetLEDs(manualColor);
        }

        // Otherwise use the default color
        else {
            forceSetLEDs(getDefaultColor());
        }
    }

    @Override
    public void forceSetLED(LEDInstruction instruction) {
        // TODO Auto-generated method stub
        
    }
}
