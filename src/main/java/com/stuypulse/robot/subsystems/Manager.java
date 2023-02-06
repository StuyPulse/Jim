package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.constants.ArmTrajectories.*;

import com.stuypulse.robot.constants.ArmTrajectories.SameSide.*;
import com.stuypulse.robot.subsystems.arm.IArm;
import com.stuypulse.robot.subsystems.intake.IIntake;
import com.stuypulse.robot.util.ArmTrajectory;

import com.stuypulse.robot.util.ArmState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manager extends SubsystemBase {

    // singleton
    private static Manager instance;

    public static Manager getInstance() {
        if (instance == null) {
            instance = new Manager();
        }
        return instance;
    }

    // game piece to score
    public enum GamePiece {
        CONE,
        CUBE
    }

    // level to score at
    public enum NodeLevel {
        HIGH, 
        MID, 
        LOW
    }

    // side to intake on
    public enum IntakeSide {
        FRONT, 
        BACK
    }

    // side to score on (relative to intake side)
    public enum ScoreSide {
        SAME,
        OPPOSITE
    }


    private GamePiece gamePiece;
    private NodeLevel nodeLevel;
    private IntakeSide intakeSide;
    private ScoreSide scoreSide;

    public Manager() {
        gamePiece = GamePiece.CUBE;
        nodeLevel = NodeLevel.HIGH;
        intakeSide = IntakeSide.FRONT;
        scoreSide = ScoreSide.SAME;
    }

    /** Generate Intake Trajectories **/

    public ArmTrajectory getIntakeTrajectory() {
        final ArmTrajectory intakeTrajectory = 
            new ArmTrajectory().addState(ArmState.fromDegrees(-60, 0));

        return intakeSide == IntakeSide.FRONT ? intakeTrajectory : intakeTrajectory.flipped();
    }

    /** Generate Ready Trajectories **/

    /** puts the trajectory on the correct side */
    /** NOTE: trajectory that score "opposite side" must have angles between -90 and -180. **/
    private ArmTrajectory normalize(ArmTrajectory trajectory) {
        boolean needsFlip = (scoreSide == ScoreSide.SAME && intakeSide == IntakeSide.BACK) ||
            (scoreSide == ScoreSide.OPPOSITE && intakeSide == IntakeSide.FRONT);

        return needsFlip ? trajectory.flipped() : trajectory;
    }
    

    public ArmTrajectory getReadyTrajectory() {
        switch (nodeLevel) {
            case LOW:
                return getIntakeTrajectory();

            case MID:
                return getMidReadyTrajectory();

            case HIGH:
                return getHighReadyTrajectory();

            default:
                return getNeutralTrajectory();
        }
    }

    private ArmTrajectory getMidReadyTrajectory() {
        switch (gamePiece) {
            case CONE:
                return normalize(ArmTrajectory.fromStates(
                        ArmState.fromDegrees(-45, 90)));

            case CUBE:
                return normalize(ArmTrajectory.fromStates(
                    ArmState.fromDegrees(0, -90)));

            default:
                return getNeutralTrajectory();
        }
    }

    private ArmTrajectory getHighReadyTrajectory() {
        switch (gamePiece) {
            case CONE:
                return normalize(ArmTrajectory.fromStates(
                        ArmState.fromDegrees(0, 90)));

            case CUBE:
                return normalize(ArmTrajectory.fromStates(
                    ArmState.fromDegrees(0, -90)));

            default:
                return getNeutralTrajectory();
        }
    }

    /** Generate Score Trajectories **/

    public ArmTrajectory getScoreTrajectory() {
        return getNeutralTrajectory();
    }

    /** Generate Neutral Trajectories **/

    public ArmTrajectory getNeutralTrajectory() {
        return ArmTrajectory.fromStates(ArmState.fromDegrees(-90, +90));
    }

    /** Change and Read State **/
    public GamePiece getGamePiece() {
        return gamePiece;
    }

    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    public NodeLevel getNodeLevel() {
        return nodeLevel;
    }

    public void setNodeLevel(NodeLevel nodeLevel) {
        this.nodeLevel = nodeLevel;
    }

    public IntakeSide getIntakeSide() {
        return intakeSide;
    }

    public void setIntakeSide(IntakeSide intakeSide) {
        this.intakeSide = intakeSide;
    }

    public ScoreSide getScoreSide() {
        return scoreSide;
    }

    public void setScoreSide(ScoreSide scoreSide) {
        this.scoreSide = scoreSide;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Manager/Game Piece", gamePiece.name());
        SmartDashboard.putString("Manager/Node Level", nodeLevel.name());
        SmartDashboard.putString("Manager/Intake Side", intakeSide.name());
        SmartDashboard.putString("Manager/Score Side", scoreSide.name());
    }
}
