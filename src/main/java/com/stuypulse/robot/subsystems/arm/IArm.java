package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.stuylib.math.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Tasks:
 * - Get Angle from arm and wrist joint
 * - Control arm and wrist joints
 * - Set target angle
 * - Go to target angle
 */
public abstract class IArm extends SubsystemBase {

    public static IArm getInstance() {
        //return RobotBase.isReal() ? new Arm() : new SimArm();
        return new Arm();
    }
    
    public abstract Angle getShoulderAngle();
    public abstract Angle getWristAngle();

    public abstract void setTargetShoulderAngle(Angle angle);
    public abstract void setTargetWristAngle(Angle angle);

    public abstract boolean isShoulderAtAngle(double maxError);
    public abstract boolean isWristAtAngle(double maxError);
}