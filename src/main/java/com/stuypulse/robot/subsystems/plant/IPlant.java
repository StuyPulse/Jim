package com.stuypulse.robot.subsystems.plant;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IPlant extends SubsystemBase {
    private static IPlant instance;

    public static IPlant getInstance() {
        if (instance == null) {
            instance = new NoPlant(); // new Plant();
        }
        return instance;
    }

    public abstract void engage();
    public abstract void disengage(); 
}