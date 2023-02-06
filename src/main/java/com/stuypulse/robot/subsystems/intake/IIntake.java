package com.stuypulse.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IIntake extends SubsystemBase {
    private static IIntake instance;
    
    public static IIntake getInstance() {
        if (instance == null) {
            instance = new NoIntake(); // new Intake();
        }
        return instance;
    }

    public abstract void acquireCube();
    public abstract void acquireCone();

    public abstract void deacquireCube();
    public abstract void deacquireCone();

    public abstract void stop();
}
