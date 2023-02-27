/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.util.StopWatch;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.LEDColor;
import com.stuypulse.robot.util.TeleopButton;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*-
 * Contains:
 *      - setColor() : sets color of LEDs for short time
 *      - getDefaultColor() : determines LED color if it is not set
 *
 * @author Sam Belliveau
 * @author Andrew Liu
 */
public class LEDController extends SubsystemBase {

    private static LEDController instance;

    public static LEDController getInstance() {
        if (instance == null) {
            instance = new LEDController();
        }
        return instance;
    }

    // Motor that controlls the LEDs
    private final PWMSparkMax controller;

    // Stopwatch to check when to start overriding manual updates
    private final StopWatch lastUpdate;
    private double manualTime;

    // The current color to set the LEDs to
    private LEDColor manualColor;

    private final Mechanism2d ledMech;
    private final MechanismLigament2d ledLigament;


    public LEDController() {
        this.controller = new PWMSparkMax(Ports.LEDController.PORT);
        this.lastUpdate = new StopWatch();

        ledMech = new Mechanism2d(2, 2);
        MechanismRoot2d root = ledMech.getRoot("LED", 0, 0);
        ledLigament = new MechanismLigament2d("LED Ligament", 2, 2);
        root.append(ledLigament);

        SmartDashboard.putData("LED Mech2d", ledMech);

        setLEDConditions();
        setColor(LEDColor.OFF);
    }

    public void setColor(LEDColor color, double time) {
        manualColor = color;
        manualTime = time;
        lastUpdate.reset();

        ledLigament.setColor(new Color8Bit(0, 0, 0));
    }

    public void setColor(LEDColor color) {
        setColor(color, Settings.LED.MANUAL_UPDATE_TIME);
    }

    private void setLEDConditions() {
        Manager manager = Manager.getInstance();

        new TeleopButton(() -> manager.getRoutine().equals(Manager.Routine.INTAKE) 
                            && manager.getGamePiece().equals(Manager.GamePiece.CONE_TIP_IN))
            .whileTrue(new LEDSet(LEDColor.YELLOW));

        new TeleopButton(() -> manager.getRoutine().equals(Manager.Routine.INTAKE) 
                            && manager.getGamePiece().equals(Manager.GamePiece.CONE_TIP_OUT))
            .whileTrue(new LEDSet(LEDColor.GREEN));

        new TeleopButton(() -> manager.getRoutine().equals(Manager.Routine.INTAKE) 
                            && manager.getGamePiece().equals(Manager.GamePiece.CUBE))
            .whileTrue(new LEDSet(LEDColor.PURPLE));

        new TeleopButton(() -> manager.getRoutine().equals(Manager.Routine.SCORE) 
                            && manager.getGamePiece().equals(Manager.GamePiece.CONE_TIP_IN))
            .whileTrue(new LEDSet(LEDColor.YELLOW.pulse()));

        new TeleopButton(() -> manager.getRoutine().equals(Manager.Routine.SCORE) 
                            && manager.getGamePiece().equals(Manager.GamePiece.CONE_TIP_OUT))
            .whileTrue(new LEDSet(LEDColor.GREEN.pulse()));

        new TeleopButton(() -> manager.getRoutine().equals(Manager.Routine.SCORE) 
                            && manager.getGamePiece().equals(Manager.GamePiece.CUBE))
            .whileTrue(new LEDSet(LEDColor.PURPLE.pulse()));

    }

    public LEDColor getDefaultColor() {
        return LEDColor.BLUE;
    }

    @Override
    public void periodic() {
        // If we called .setColor() recently, use that value
        if (DriverStation.isAutonomous() || lastUpdate.getTime() < manualTime) {
            controller.set(manualColor.get());
        }

        // Otherwise use the default color
        else {
            controller.set(getDefaultColor().get());
        }
    }
}
