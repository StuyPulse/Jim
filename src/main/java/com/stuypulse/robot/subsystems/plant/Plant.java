package com.stuypulse.robot.subsystems.plant;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Plant extends SubsystemBase {
    private static Plant instance;

    public static Plant getInstance() {
        if (instance == null) {
            instance = Settings.ROBOT == Robot.JIM ? new PlantImpl() : new NoPlant();;
        }
        return instance;
    }

    public abstract void engage();
    public abstract void disengage(); 
}