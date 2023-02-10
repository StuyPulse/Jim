package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmVisualizer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {

    private static Arm instance = null;

    public static Arm getInstance() {
        if (instance == null) {
            instance = RobotBase.isSimulation() ? new SimArm() : new ArmImpl();
        }
        return instance;
    }
    
    public abstract Rotation2d getShoulderAngle();
    public abstract Rotation2d getWristAngle();

    public final ArmState getState() {
        return new ArmState(getShoulderAngle(), getWristAngle());
    }

    public abstract Rotation2d getShoulderTargetAngle();
    public abstract Rotation2d getWristTargetAngle();

    public final boolean isShoulderAtAngle(Rotation2d maxError) {
        return Math.abs(getShoulderTargetAngle().minus(getShoulderAngle()).getDegrees()) < maxError.getDegrees();
    }
    public final boolean isWristAtAngle(Rotation2d maxError) {
        return Math.abs(getWristTargetAngle().minus(getWristAngle()).getDegrees()) < maxError.getDegrees();
    }

    public final boolean isArmAtState(Rotation2d shoulderEpsilon, Rotation2d wristEpsilon) {
        return isShoulderAtAngle(shoulderEpsilon) && isWristAtAngle(wristEpsilon);
    }

    public abstract void setTargetShoulderAngle(Rotation2d angle);
    public abstract void setTargetWristAngle(Rotation2d angle);

    public final void setTargetState(ArmState state) {
        setTargetShoulderAngle(state.getShoulderState());
        setTargetWristAngle(state.getWristState());
    }

    public final void moveShoulder(Rotation2d angle) {
        setTargetShoulderAngle(getShoulderTargetAngle().plus(angle));
    }

    public final void moveWrist(Rotation2d angle) {
        setTargetWristAngle(getWristTargetAngle().plus(angle));
    }

    public abstract ArmVisualizer getVisualizer();
}
