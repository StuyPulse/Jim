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
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDVisualizer {
    // Simulation Constants
    private final double WINDOW_WIDTH = 55;
    private final double WINDOW_HEIGHT = 10;
    private final double WINDOW_X_PADDING = 0.5;
    private final double LED_WIDTH = 15;
    
    // Mechanism2ds that displays the LEDs
    private final Mechanism2d sim;
    private final MechanismRoot2d startRoot;
    private final MechanismLigament2d[] leds;
    private final AddressableLEDBuffer ledsBuffer;

    // Stopwatch to check when to start overriding manual updates
    private StopWatch lastUpdate;
    private double manualTime;
    
    // The current color to set the LEDs to
    private LEDColor manualColor;

    protected LEDVisualizer() {
        sim = new Mechanism2d(WINDOW_WIDTH, WINDOW_HEIGHT);
        leds = new MechanismLigament2d[Settings.LED.LED_LENGTH];
        ledsBuffer = new AddressableLEDBuffer(Settings.LED.LED_LENGTH);

        // Create a row of squares to represent the LEDs
        for (int i = 0; i < Settings.LED.LED_LENGTH; i++) {
            leds[i] = new MechanismLigament2d("LED " + i, WINDOW_X_PADDING + i, 0, LED_WIDTH, new Color8Bit(0, 0, 0));
        }

        // Set the root of the Mechanism2d to be the center of the row of squares
        startRoot = sim.getRoot("Led Root", 0, WINDOW_HEIGHT / 2);

        // Append the squares onto each other
        for (int i = 0; i < Settings.LED.LED_LENGTH; i++) {
            startRoot.append(leds[i]);
        }
        
        // Set the LEDs to be off
        forceSetLED(LEDColor.BLACK);

        // Initialize the stopwatch
        lastUpdate = new StopWatch();

        SmartDashboard.putData("SimLED", sim);

    }

    public void setColor(LEDColor color, double time) {
        manualColor = color;
        manualTime = time;
        lastUpdate.reset();
    }

    private void forceSetLED(LEDColor ledColor) {
        for (int i = 0; i < ledsBuffer.getLength(); i++) {
            ledsBuffer.setRGB(i, ledColor.getRed(), ledColor.getGreen(), ledColor.getBlue());
        }
    }

    public void forceSetLED(LEDInstruction instruction) {
        // for (int i = 0; i < ledsBuffer.getLength(); i++) {
        //     ledsBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        // }
        // leds.setData(ledsBuffer);
        instruction.setLED(ledsBuffer);
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
     
    public void periodic() {
        // If we called .setColor() recently, use that value
        if (Robot.getMatchState() == MatchState.AUTO || lastUpdate.getTime() < manualTime) {
            forceSetLED(manualColor);
            SmartDashboard.putNumber("LED Mech2d/LED 0R", manualColor.getRed());
            SmartDashboard.putNumber("LED Mech2d/LED 0G", manualColor.getGreen());
            SmartDashboard.putNumber("LED Mech2d/LED 0B", manualColor.getBlue());
        }

        // Otherwise use the default color
        else {
            //forceSetLED(getDefaultColor());
            forceSetLED(LEDColor.RAINBOW);
        }

        SmartDashboard.putNumber("LED Mech2d/LED Time", lastUpdate.getTime());
    }
}
