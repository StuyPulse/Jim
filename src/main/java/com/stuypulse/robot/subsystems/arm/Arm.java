package com.stuypulse.robot.subsystems.arm;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.robot.util.ArmBFSField;
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
                instance = new SimArm();
            else if (Settings.ROBOT == Robot.JIM)
                instance = new ArmImpl();
            else
                instance = new PerfectArm();
        }
        return instance;
    }
    
    // Path planning
    private Optional<ArmBFSField> trajectory = Optional.empty();
    private ArmState targetState = ArmState.fromDegrees(-90, +90);

    // Read measured state
    public abstract Rotation2d getShoulderAngle();
    public abstract Rotation2d getWristAngle();

    public final ArmState getState() {
        return new ArmState(getShoulderAngle(), getWristAngle());
    }

    // Read target state
    private final ArmBFSField.Node getTargetNode() {
        return trajectory.get().getNode(getState()).travel(Settings.Arm.BFS_FIELD_LEAD.getAsDouble());
    }

    public final ArmState getTargetState() {
        if(trajectory.isPresent()) {
            targetState = getTargetNode().getArmState();
        }

        return targetState;
    }

    public final Rotation2d getShoulderTargetAngle() {
        return getTargetState().getShoulderState();
    }
    public final Rotation2d getWristTargetAngle() {
        return getTargetState().getWristState();
    }
    
    // Read error between measured and target states
    public final boolean isShoulderAtAngle(Rotation2d maxError) {
        return Math.abs(getShoulderTargetAngle().minus(getShoulderAngle()).getDegrees()) < maxError.getDegrees();
    }
    public final boolean isWristAtAngle(Rotation2d maxError) {
        return Math.abs(getWristTargetAngle().minus(getWristAngle()).getDegrees()) < maxError.getDegrees();
    }

    public final boolean isArmAtTargetState(Rotation2d shoulderEpsilon, Rotation2d wristEpsilon) {
        return isShoulderAtAngle(shoulderEpsilon) && isWristAtAngle(wristEpsilon);
    }

    public final boolean isArmAtEndState(Rotation2d shoulderEpsilon, Rotation2d wristEpsilon) {
        // Checks if the current target state is the goal / end point of a trajectory
        // -- this is what is meant by "end" state
        if(trajectory.isPresent() && !getTargetNode().isSetpoint()) {
            return false;
        }

        return isArmAtTargetState(shoulderEpsilon, wristEpsilon);
    }

    // Set target state
    public final void setTargetState(ArmState state) {
        trajectory = Optional.empty();
        targetState = state;
    }

    public final void setShoulderTargetState(Rotation2d shoulder) {
        setTargetState(new ArmState(shoulder, getWristTargetAngle()));
    }

    public final void setWristTargetState(Rotation2d wrist) {
        setTargetState(new ArmState(getShoulderTargetAngle(),wrist));
    }

    public final void setTrajectory(ArmBFSField trajectory) {
        this.trajectory = Optional.ofNullable(trajectory);
    }

    // Change target angle (useful for driving)
    public final void moveShoulder(Rotation2d angle) {
        setShoulderTargetState(getShoulderTargetAngle().plus(angle));
    }

    public final void moveWrist(Rotation2d angle) {
        setWristTargetState(getWristTargetAngle().plus(angle));
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
