package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.subsystems.arm.Arm;
// import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.SimArm;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.wpilibj.RobotBase;
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
        return RobotBase.isReal() ? new Arm() : new SimArm();
    }
    
    public abstract double getShoulderDegrees();
    public abstract double getWristDegrees();

    public abstract void setTargetShoulderAngle(double degrees);
    public abstract void setTargetWristAngle(double degrees);
    public abstract void setTargetWristAngle(double degrees, boolean clockwise);

    public abstract boolean isShoulderAtAngle(double maxError);
    public abstract boolean isWristAtAngle(double maxError);
    
}
