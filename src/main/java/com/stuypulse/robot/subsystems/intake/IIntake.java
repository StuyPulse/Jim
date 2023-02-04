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
    
    public abstract void cubeIntake();
    public abstract void coneIntake();
    public abstract void cubeOuttake();
    public abstract void coneOuttake();

    public abstract void stop();
}
