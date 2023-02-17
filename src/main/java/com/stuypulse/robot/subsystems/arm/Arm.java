package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmVisualizer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {

    // Singleton
    private static Arm instance = null;

    public static Arm getInstance() {
        if (instance == null) {
            if (RobotBase.isSimulation())
                instance = new PerfectSimArm();
            else if (Settings.ROBOT == Robot.JIM)
                instance = new ArmImpl();
            else
                instance = new PerfectArm();
        }
        return instance;
    }
    
    // Read arm state
    public abstract Rotation2d getShoulderAngle();
    public abstract Rotation2d getWristAngle();

    public final ArmState getState() {
        return new ArmState(getShoulderAngle(), getWristAngle());
    }

    // Read target state
    public abstract Rotation2d getShoulderTargetAngle();
    public abstract Rotation2d getWristTargetAngle();

    public final ArmState getTargetState() {
        return new ArmState(getShoulderTargetAngle(), getWristTargetAngle());
    }

    // Compare measurement and target
    public final boolean isShoulderAtAngle(Rotation2d maxError) {
        return Math.abs(getShoulderTargetAngle().minus(getShoulderAngle()).getDegrees()) < maxError.getDegrees();
    }
    public final boolean isWristAtAngle(Rotation2d maxError) {
        return Math.abs(getWristTargetAngle().minus(getWristAngle()).getDegrees()) < maxError.getDegrees();
    }

    public final boolean isArmAtState(Rotation2d shoulderEpsilon, Rotation2d wristEpsilon) {
        return isShoulderAtAngle(shoulderEpsilon) && isWristAtAngle(wristEpsilon);
    }

    // Set target state
    public abstract void setTargetShoulderAngle(Rotation2d angle);
    public abstract void setTargetWristAngle(Rotation2d angle);

    public final void setTargetState(ArmState state) {
        setTargetShoulderAngle(state.getShoulderState());
        setTargetWristAngle(state.getWristState());
    }

    // Change target angle
    public final void moveShoulder(Rotation2d angle) {
        setTargetShoulderAngle(getShoulderTargetAngle().plus(angle));
    }

    public final void moveWrist(Rotation2d angle) {
        setTargetWristAngle(getWristTargetAngle().plus(angle));
    }

    // Enable feedback control
    public abstract void setFeedbackEnabled(boolean enabled);

    public final void enableFeedback() {
        setFeedbackEnabled(true);
    }

    public final void disableFeedback() {
        setFeedbackEnabled(false);
    }

    // Get arm visualizer
    public abstract ArmVisualizer getVisualizer();
}
