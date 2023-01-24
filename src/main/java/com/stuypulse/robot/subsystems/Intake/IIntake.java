package com.stuypulse.robot.subsystems.Intake;
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
    public abstract void cubeIntake();
    public abstract void coneIntake();
    public abstract void cubeOuttake();
    public abstract void coneOuttake();
    public abstract boolean isStalling();
    public abstract double getAngle();

}
