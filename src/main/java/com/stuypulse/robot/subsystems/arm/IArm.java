package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
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
        return RobotBase.isReal() ? new SimArm() : new SimArm();
    }
    
    public abstract Rotation2d getShoulderAngle();
    public abstract Rotation2d getWristAngle();

    public abstract void setTargetShoulderAngle(double degrees);
    public final void setTargetWristAngle(double degrees) {
        setTargetWristAngle(degrees, false);
    }
    public abstract void setTargetWristAngle(double degrees, boolean longPath);

    public abstract boolean isShoulderAtAngle(double maxError);
    public abstract boolean isWristAtAngle(double maxError);
    
}
