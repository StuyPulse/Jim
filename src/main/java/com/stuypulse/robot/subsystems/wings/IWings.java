package com.stuypulse.robot.subsystems.wings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IWings extends SubsystemBase {

    private static IWings instance;

    public static IWings getInstance() {
        if (instance == null) {
            instance = new NoWings(); // new Wings();
        }
        return instance;
    }


    public abstract void extendLeft();
    public abstract void retractLeft();
    public abstract void extendRight();
    public abstract void retractRight();
}
