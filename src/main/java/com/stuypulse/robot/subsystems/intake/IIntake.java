package com.stuypulse.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IIntake extends SubsystemBase {
    private static Intake instance;
    
    public enum GamePiece{
        cone,
        cube;
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }
    public abstract void cubeIntake();
    public abstract void coneIntake();
    public abstract void cubeOuttake();
    public abstract void coneOuttake();
    public abstract void stop();
}
