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
                                    case READY:
                                    case SCORE:
                                    case NEUTRAL:
                                }
                            case CUBE: 
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY:
                                    case SCORE:
                                    case NEUTRAL:
                                }
                        }
                        case MID:
                        switch (currentPiece) {
                            case CONE:
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY:
                                    case SCORE:
                                    case NEUTRAL:
                                }
                            case CUBE: 
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY:
                                    case SCORE:
                                    case NEUTRAL:
                                }
                        }
                        case LOW:
                        switch (currentPiece) {
                            case CONE:
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY:
                                    case SCORE:
                                    case NEUTRAL:
                                }
                            case CUBE: 
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY:
                                    case SCORE:
                                    case NEUTRAL:
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
                                    case READY:
                                    case SCORE:
                                    case NEUTRAL:
                                }
                            case CUBE: 
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY:
                                    case SCORE:
                                    case NEUTRAL:
                                }
                        }
                        case MID:
                        switch (currentPiece) {
                            case CONE:
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY:
                                    case SCORE:
                                    case NEUTRAL:
                                }
                            case CUBE: 
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY:
                                    case SCORE:
                                    case NEUTRAL:
                                }
                        }
                        case LOW:
                        switch (currentPiece) {
                            case CONE:
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY:
                                    case SCORE:
                                    case NEUTRAL:
                                }
                            case CUBE: 
                                switch (currentMode) {
                                    case INTAKE:
                                    case READY:
                                    case SCORE:
                                    case NEUTRAL:
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
