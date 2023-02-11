package com.stuypulse.robot.subsystems.wings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Wings extends SubsystemBase {

    private static Wings instance;

    public static Wings getInstance() {
        if (instance == null) {
            // instance = new WingsImpl();
            instance = new NoWings();
        }
        return instance;
    }


    public abstract void extendLeft();
    public abstract void retractLeft();
    public abstract void extendRight();
    public abstract void retractRight();
}
