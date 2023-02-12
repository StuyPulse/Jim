package com.stuypulse.robot.subsystems.plant;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Plant extends SubsystemBase {
    private static Plant instance;

    public static Plant getInstance() {
        if (instance == null) {
            instance = new PlantImpl();
        }
        return instance;
    }

    public abstract void engage();
    public abstract void disengage(); 
}