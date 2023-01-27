package com.stuypulse.robot.subsystems.vision;

import java.util.List;

import com.stuypulse.robot.subsystems.vision.Vision.Result;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IVision extends SubsystemBase {

    private static IVision instance;

    public static IVision getInstance() {
        if(instance == null) {
            instance = new Vision();
        } 
        return instance; 
    }
    
    private enum Noise {
        LOW,
        MID,
        HIGH;
    }

    public abstract List<Result> getResults();
}
