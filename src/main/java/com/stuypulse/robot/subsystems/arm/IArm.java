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
            return new Arm();
        }
        return getInstance();
    }
    
    public abstract Rotation2d getShoulderAngle();
    public abstract Rotation2d getWristAngle();

<<<<<<< HEAD
    public abstract void setTargetShoulderAngle(Rotation2d angle);
    public final void setTargetWristAngle(Rotation2d angle) {
        setTargetWristAngle(angle, false);
    }
    public abstract void setTargetWristAngle(Rotation2d angle, boolean longPath);
=======
    public abstract void setTargetShoulderAngle(double degrees);
    public abstract void setTargetWristAngle(double degrees);
>>>>>>> a1cc62a80922ef349e0a331aec0afd135a26a4de

    public abstract boolean isShoulderAtAngle(double maxError);
    public abstract boolean isWristAtAngle(double maxError);

    public abstract Rotation2d getShoulderTargetAngle();
    public abstract Rotation2d getWristTargetAngle();

    public final void addShoulderAngle(Rotation2d angle) {
        setTargetShoulderAngle(new Rotation2d(getShoulderAngle().getDegrees() + angle.getDegrees()));
    }

    public final void addWristAngle(Rotation2d angle) {
        setTargetWristAngle(new Rotation2d(getWristAngle().getDegrees() + angle.getDegrees()));
    }
}
