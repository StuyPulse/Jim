package com.stuypulse.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IIntake extends SubsystemBase {
    private static Intake instance;
    
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public abstract void acquireCube();
    public abstract void acquireCone();

    public abstract void deacquireCube();
    public abstract void deacquireCone();

    public abstract void stop();
}
