package com.stuypulse.robot.subsystems.intake;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    private static Intake instance;
    
    public static Intake getInstance() {
        if (instance == null) {
            instance = Settings.ROBOT == Robot.JIM ? new IntakeImpl() : new NoIntake();
        }
        return instance;
    }

    public abstract void acquireCube();
    public abstract void acquireCone();

    public abstract void deacquireCube();
    public abstract void deacquireCone();

    public abstract void stop();

    public abstract boolean hasNewGamePiece();
}
