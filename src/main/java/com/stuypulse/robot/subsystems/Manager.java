package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.constants.ArmTrajectories.*;

import com.stuypulse.robot.constants.ArmTrajectories.SameSide.*;
import com.stuypulse.robot.util.ArmTrajectory;

public class Manager {

    private static Manager instance;

    public static Manager getInstance() {
        if (instance == null) {
            instance = new Manager();
        }
        return instance;
    }

    public enum Piece {
        CONE,
        CUBE
    }

    public enum Level {
        HIGH, 
        MID, 
        LOW
    }

    public enum Side {
        SAME,
        OPPOSITE
    }

    public enum Mode {
        INTAKE,
        READY,
        SCORE,
        NEUTRAL
    }

    private Piece piece;
    private Level level;
    private Side side;
    private Mode mode;

    public Manager() {
        this.piece = Piece.CONE;
        this.level = Level.HIGH;
        this.side = Side.SAME;
        this.mode = Mode.NEUTRAL;
    }

    public Level getLevel() {
        return level;
    }

    public Piece getPiece() {
        return piece;
    }

    public Side getSide() {
        return side;
    }

    public Mode getMode() {
        return mode;
    }

    public ArmTrajectory append(ArmTrajectory path, ArmTrajectory pathToAppend) {
        return path.addState(pathToAppend);
    }

    public ArmTrajectory toHome() {
        return NEUTRAL;
    }

    public ArmTrajectory switchSides(ArmTrajectory path) {
        return path.switchSides();
    }

    public ArmTrajectory switchConeHigh() {
        System.out.println("hello");
        switch (mode) {
            case INTAKE: 
            case READY: return High.Cone.READY;
            case SCORE: return High.Cone.SCORE;
            case NEUTRAL: return High.Cone.SCORE_TO_NEUTRAL;
        }
        return NEUTRAL;
    }
    public ArmTrajectory switchConeMid() {
        switch (mode) {
            case INTAKE: 
            case READY: return Mid.Cone.READY;
            case SCORE: return Mid.Cone.SCORE;
            case NEUTRAL: return Mid.Cone.SCORE_TO_NEUTRAL;
        }
        return NEUTRAL;
    }
    public ArmTrajectory switchConeLow() {
        switch (mode) {
            case INTAKE: 
            case READY: return Low.Cone.READY;
            case SCORE: return Low.Cone.SCORE;
            case NEUTRAL: return Low.Cone.SCORE_TO_NEUTRAL;
        }
        return NEUTRAL;
    }

    public ArmTrajectory switchCubeHigh() {
        switch (mode) {
            case INTAKE:
            case READY: return High.Cube.READY;
            case SCORE: return High.Cube.SCORE;
            case NEUTRAL: return High.Cube.SCORE_TO_NEUTRAL;
        }
        return NEUTRAL;
    }
    public ArmTrajectory switchCubeMid() {
        switch (mode) {
            case INTAKE:
            case READY: return Mid.Cube.READY;
            case SCORE: return Mid.Cube.SCORE;
            case NEUTRAL: return Mid.Cube.SCORE_TO_NEUTRAL;
        }
        return NEUTRAL;
    }
    public ArmTrajectory switchCubeLow() {
        switch (mode) {
            case INTAKE:
            case READY: return Low.Cube.READY;
            case SCORE: return Low.Cube.SCORE;
            case NEUTRAL: return Low.Cube.SCORE_TO_NEUTRAL;
        }
        return NEUTRAL;
    }

    public ArmTrajectory switchCone() {
        switch (level) {
            case HIGH: return switchConeHigh();
            case MID: return switchConeMid();
            case LOW: return switchConeLow();
            default: return NEUTRAL;
        }
    }
    public ArmTrajectory switchCube() {
        switch (level) {
            case HIGH: return switchCubeHigh();
            case MID: return switchCubeMid();
            case LOW: return switchCubeLow();
            default: return NEUTRAL;
        }
    }

    public ArmTrajectory getTrajectory(Side side, Mode mode) {
        this.side = side;
        this.mode = mode;

        switch (side) {
            case SAME:
                switch (piece) {
                    case CONE: return switchCone();
                    case CUBE: return switchCube();
                }
            case OPPOSITE:
                switch (piece) {
                    case CONE: return switchCone().switchSides();
                    case CUBE: return switchCube().switchSides();
                }
            default: return NEUTRAL;
        }        
    }

    public void setLevel(Level level) {
        this.level = level;
    }

    public void setPiece(Piece piece) {
        this.piece = piece;
    }

    public void setSide(Side side) {
        this.side = side;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }
}
