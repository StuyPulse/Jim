package com.stuypulse.robot.subsystems.intake;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {

    // Singleton
    private static final Intake instance;

    static {
        if (RobotBase.isSimulation())
            instance = new SimIntake();
        else if (Settings.ROBOT == Robot.JIM)
            instance = new IntakeImpl();
        else
            instance = new SimIntake();
    }
    
    public static Intake getInstance() {
        return instance;
    }

    // Intake methods
    protected Intake() {
    }

    public abstract void acquire();
    public abstract void deacquire();
    public abstract void stop();

    public boolean hasGamePiece() {return false;}

    public void enableCoast() { }
    public void enableBreak() { }
}
