package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.constants.ArmTrajectories.*;
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

    private Piece currentPiece;
    private Level currentLevel;
    private Side currentSide;
    private Mode currentMode;

    public Manager() {
        this.currentPiece = Piece.CONE;
        this.currentLevel = Level.HIGH;
        this.currentSide = Side.SAME;
        this.currentMode = Mode.NEUTRAL;
    }

    public Level getLevel() {
        return currentLevel;
    }

    public Piece getPiece() {
        return currentPiece;
    }

    public Side getSide() {
        return currentSide;
    }

    public Mode getMode() {
        return currentMode;
    }

    public ArmTrajectory append(ArmTrajectory path, ArmTrajectory pathToAppend) {
        return path.addState(pathToAppend);
    }

    public ArmTrajectory getPath(Side side, Mode mode) {
        currentSide = side;
        currentMode = mode;

        switch (currentSide) {
            case SAME:
                switch (currentLevel) {
                    case HIGH:
                        switch (currentPiece) {
                            case CONE:
                                switch (currentMode) {
                                    case INTAKE: 
                                    case READY: return SameSide.High.Cone.READY;
                                    case SCORE: return SameSide.High.Cone.SCORE;
                                    case NEUTRAL: return SameSide.High.Cone.SCORE_TO_NEUTRAL;
                                }
                            case CUBE: 
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY: return SameSide.High.Cube.READY;
                                    case SCORE: return SameSide.High.Cube.SCORE;
                                    case NEUTRAL: return SameSide.High.Cube.SCORE_TO_NEUTRAL;
                                }
                        }
                        case MID:
                        switch (currentPiece) {
                            case CONE:
                                switch (currentMode) {
                                    case INTAKE: 
                                    case READY: return SameSide.Mid.Cone.READY;
                                    case SCORE: return SameSide.Mid.Cone.SCORE;
                                    case NEUTRAL: return SameSide.Mid.Cone.SCORE_TO_NEUTRAL;
                                }
                            case CUBE: 
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY: return SameSide.Mid.Cube.READY;
                                    case SCORE: return SameSide.Mid.Cube.SCORE;
                                    case NEUTRAL: return SameSide.Mid.Cube.SCORE_TO_NEUTRAL;
                                }
                        }
                        case LOW:
                        switch (currentPiece) {
                            case CONE:
                                switch (currentMode) {
                                    case INTAKE: 
                                    case READY: return SameSide.Low.Cone.READY;
                                    case SCORE: return SameSide.Low.Cone.SCORE;
                                    case NEUTRAL: return SameSide.Low.Cone.SCORE_TO_NEUTRAL;
                                }
                            case CUBE: 
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY: return SameSide.Low.Cube.READY;
                                    case SCORE: return SameSide.Low.Cube.SCORE;
                                    case NEUTRAL: return SameSide.Low.Cube.SCORE_TO_NEUTRAL;
                                }
                        }
                }
                case OPPOSITE:
                switch (currentLevel) {
                    case HIGH:
                        switch (currentPiece) {
                            case CONE:
                                switch (currentMode) {
                                    case INTAKE: 
                                    case READY: return OppositeSide.High.Cone.READY;
                                    case SCORE: return OppositeSide.High.Cone.SCORE;
                                    case NEUTRAL: return OppositeSide.High.Cone.SCORE_TO_NEUTRAL;
                                }
                            case CUBE: 
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY: return OppositeSide.High.Cube.READY;
                                    case SCORE: return OppositeSide.High.Cube.SCORE;
                                    case NEUTRAL: return OppositeSide.High.Cube.SCORE_TO_NEUTRAL;
                                }
                        }
                        case MID:
                        switch (currentPiece) {
                            case CONE:
                                switch (currentMode) {
                                    case INTAKE: 
                                    case READY: return OppositeSide.Mid.Cone.READY;
                                    case SCORE: return OppositeSide.Mid.Cone.SCORE;
                                    case NEUTRAL: return OppositeSide.Mid.Cone.SCORE_TO_NEUTRAL;
                                }
                            case CUBE: 
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY: return OppositeSide.Mid.Cube.READY;
                                    case SCORE: return OppositeSide.Mid.Cube.SCORE;
                                    case NEUTRAL: return OppositeSide.Mid.Cube.SCORE_TO_NEUTRAL;
                                }
                        }
                        case LOW:
                        switch (currentPiece) {
                            case CONE:
                                switch (currentMode) {
                                    case INTAKE: 
                                    case READY: return OppositeSide.Low.Cone.READY;
                                    case SCORE: return OppositeSide.Low.Cone.SCORE;
                                    case NEUTRAL: return OppositeSide.Low.Cone.SCORE_TO_NEUTRAL;
                                }
                            case CUBE: 
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY: return OppositeSide.Low.Cube.READY;
                                    case SCORE: return OppositeSide.Low.Cube.SCORE;
                                    case NEUTRAL: return OppositeSide.Low.Cube.SCORE_TO_NEUTRAL;
                                }
                        }
                }
            default:
                return NEUTRAL;
        }    
    }

    public void setLevel(Level level) {
        currentLevel = level;
    }

    public void setPiece(Piece piece) {
        currentPiece = piece;
    }

    public void setSide(Side side) {
        currentSide = side;
    }

    public void setMode(Mode mode) {
        currentMode = mode;
    }
}
