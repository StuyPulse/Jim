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

    private static IArm instance;

    public static IArm getInstance() {
        if (instance == null) {
            if (RobotBase.isSimulation()) return new SimArm();
            return new Arm();
        }
        return getInstance();
    }
    
    public abstract Rotation2d getShoulderAngle();
    public abstract Rotation2d getWristAngle();

    public abstract void setTargetShoulderAngle(Rotation2d angle);
    public abstract void setTargetWristAngle(Rotation2d angle);

    public abstract boolean isShoulderAtAngle(Rotation2d maxError);
    public abstract boolean isWristAtAngle(Rotation2d maxError);

    public abstract Rotation2d getShoulderTargetAngle();
    public abstract Rotation2d getWristTargetAngle();

    public final void addShoulderAngle(Rotation2d angle) {
        setTargetShoulderAngle(new Rotation2d(getShoulderAngle().getDegrees() + angle.getDegrees()));
    }

    public final void addWristAngle(Rotation2d angle) {
        setTargetWristAngle(new Rotation2d(getWristAngle().getDegrees() + angle.getDegrees()));
    }

    public final void moveShoulder(Rotation2d angle) {
        setTargetShoulderAngle(getShoulderTargetAngle().plus(angle));
    }

    public final void moveWrist(Rotation2d angle) {
        setTargetWristAngle(getWristTargetAngle().plus(angle));
    }
}
