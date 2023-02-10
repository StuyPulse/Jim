package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.util.ArmState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {

    private static Arm instance = null;

    public static Arm getInstance() {
        if (instance == null) {
            // instance = RobotBase.isSimulation() ? new SimArm() : new ArmImpl();
            instance = new NoArm();
        }
        return instance;
    }
    
    public abstract Rotation2d getShoulderAngle();
    public abstract Rotation2d getWristAngle();

    public abstract Rotation2d getShoulderTargetAngle();
    public abstract Rotation2d getWristTargetAngle();

    public abstract boolean isShoulderAtAngle(Rotation2d maxError);
    public abstract boolean isWristAtAngle(Rotation2d maxError);

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
}
