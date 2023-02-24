package com.stuypulse.robot.subsystems.intake;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    private static final Intake instance;

    static {
        if (RobotBase.isSimulation())
            instance = new SimIntake();
        else if (Settings.ROBOT == Robot.JIM)
            instance = new IntakeImpl();
        else
            instance = new SimIntake();
        // instance = new SimIntake();
    }
    
    public static Intake getInstance() {
        return instance;
    }

    public abstract void acquireCube();
    public abstract void acquireCone();

    public abstract void deacquireCube();
    public abstract void deacquireCone();

    public abstract void stop();

    public abstract boolean hasNewGamePiece();
}
