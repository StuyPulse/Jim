package com.stuypulse.robot.subsystems.intake;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * fields: 
 * - 2 motors
 * 
 * methods:
 * - cube intake
 * - cone intake
 * - cube outtake
 * - cone outtake
 * - stalling
 * - getangle (of arm) to know if intake is flipped
 * 
 * ports
 * - 2 motors
 * - IR sensor
 * 
 * Contants
 * - Stalling Constant
 * 
 * periodic:
 * - get angle (smartnumber)
 * - isStalling (smartnumber)
 * - is IR sensor tripped smartnumber
 */

public abstract class IIntake extends SubsystemBase {
    //shut up amber
    private static IIntake instance;
    public static IIntake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }
    public abstract void cubeIntake();
    public abstract void coneIntake();
    public abstract void cubeOuttake();
    public abstract void coneOuttake();
}
