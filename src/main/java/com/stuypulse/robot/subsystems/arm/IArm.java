package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IArm extends SubsystemBase {

    private static IArm instance = null;

    public static IArm getInstance() {
        if (instance == null) {
            instance = new NoArm(); // RobotBase.isSimulation() ? new SimArm() : new Arm();
        }
        return instance;
    }
    
    public abstract Rotation2d getShoulderAngle();
    public abstract Rotation2d getWristAngle();

    public abstract Rotation2d getShoulderTargetAngle();
    public abstract Rotation2d getWristTargetAngle();

    public abstract boolean isShoulderAtAngle(Rotation2d maxError);
    public abstract boolean isWristAtAngle(Rotation2d maxError);

    public abstract void setTargetShoulderAngle(Rotation2d angle);
    public abstract void setTargetWristAngle(Rotation2d angle);

    public final void moveShoulder(Rotation2d angle) {
        setTargetShoulderAngle(getShoulderTargetAngle().plus(angle));
    }

    public final void moveWrist(Rotation2d angle) {
        setTargetWristAngle(getWristTargetAngle().plus(angle));
    }
}
