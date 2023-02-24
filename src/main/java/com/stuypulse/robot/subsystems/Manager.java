package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.ArmTrajectories.*;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        CONE_TIP_IN(false),
        CONE_TIP_UP(false),
        CUBE(true);

        private final boolean cube;
        private GamePiece(boolean cube) {
            this.cube = cube;
        }

        public boolean isCube() {
            return cube;
        }

        public boolean isCone() {
            return !isCube();
        }
    }

    // level to score at
    public enum NodeLevel {
        HIGH, 
        MID, 
        LOW
    }

    // side to score on (relative to intake side)
    public enum ScoreSide {
        FRONT,
        BACK
    }

    // Direction that describes a scoring position
    public enum Direction {
        LEFT,
        CENTER,
        RIGHT
    }

    private GamePiece gamePiece;
    private NodeLevel nodeLevel;
    private ScoreSide scoreSide;

    private Direction gridSection;
    private Direction gridColumn;

    public Manager() {
        gamePiece = GamePiece.CUBE;
        nodeLevel = NodeLevel.HIGH;
        scoreSide = ScoreSide.FRONT;

        gridSection = Direction.CENTER;
        gridColumn = Direction.CENTER;
    }

    /** Generate Intake Trajectories **/


    public ArmState getIntakeTrajectory() {
        if (gamePiece.isCone())
            return Acquire.kCone;
        else
            return Acquire.kCube;
    }

    public ArmState getOuttakeTrajectory() {
        return Deacquire.kTrajectory;
    }

    /** Generate Ready Trajectories **/

    public ArmState getReadyTrajectory() {
        switch (nodeLevel) {
            case LOW:
                return getLowReadyTrajectory();

            case MID:
                return getMidReadyTrajectory();

            case HIGH:
                return getHighReadyTrajectory();

            default:
                return getNeutralTrajectory();
        }
    }

    private ArmState getLowReadyTrajectory() {
        return Deacquire.kTrajectory;
    }

    private ArmState getMidReadyTrajectory() {
        switch (gamePiece) {
            // case CONE_TIP_OUT:
            //     return Ready.Mid.kConeTipOutFront;

            case CONE_TIP_IN:
                return Ready.Mid.kConeTipInBack;

            case CUBE:
                return scoreSide == ScoreSide.FRONT ? Ready.Mid.kCubeFront : Ready.Mid.kCubeBack;

            default:
                return getNeutralTrajectory();
        }
    }

    private ArmState getHighReadyTrajectory() {
        switch (gamePiece) {
            case CONE_TIP_IN:
                return Ready.High.kConeTipInBack;

            case CONE_TIP_UP:
                return Ready.High.kConeTipUpBack;
                
            case CUBE:
                return scoreSide == ScoreSide.FRONT ? Ready.High.kCubeFront : Ready.High.kCubeBack;

            default:
                return getNeutralTrajectory();
        }
    }

    /** Generate Score Trajectories **/

    public ArmState getScoreTrajectory() {
        switch (nodeLevel) {
            case LOW:
                return getLowReadyTrajectory();

            case MID:
                if (gamePiece == GamePiece.CUBE)
                    return scoreSide == ScoreSide.FRONT ? Score.Mid.kCubeFront : Score.Mid.kCubeBack;
                
                // if (gamePiece == GamePiece.CONE_TIP_OUT)
                //     return Score.Mid.kConeTipOutFront;

                return Score.Mid.kConeTipInBack;

            case HIGH:
                if (gamePiece == GamePiece.CUBE)
                    return scoreSide == ScoreSide.FRONT ? Score.High.kCubeFront : Score.High.kCubeBack;

                else if (gamePiece == GamePiece.CONE_TIP_UP)
                    return Score.High.kConeTipUpBack;

                return Score.High.kConeTipInBack;

            default:
                return getNeutralTrajectory();
        }
    }

    /** Generate Neutral Trajectories **/

    // wrist faces away from scoring direction for cube
    public ArmState getNeutralTrajectory() {
        return Neutral.kTrajectory;
    }

    /** Generate Score Pose **/

    public Translation2d getScoreTranslation() {
        int index = gridSection.ordinal() * 3 + gridColumn.ordinal();
        
        if (RobotContainer.getCachedAlliance() == Alliance.Blue) {
            return Field.BLUE_ALIGN_POSES[index];
        } else {
            return Field.RED_ALIGN_POSES[index];
        }
    }

    public Pose2d getScorePose() {
        Rotation2d rotation = new Rotation2d();

        if (scoreSide == ScoreSide.FRONT)
            rotation = Rotation2d.fromDegrees(180);

        return new Pose2d(getScoreTranslation(), rotation);
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

    public ScoreSide getScoreSide() {
        return scoreSide;
    }

    public void setScoreSide(ScoreSide scoreSide) {
        this.scoreSide = scoreSide;
    }

    public Direction getGridSection() {
        return gridSection;
    }

    public void setGridSection(Direction gridSection) {
        this.gridSection = gridSection;
    }

    public Direction getGridColumn() {
        return gridColumn;
    }

    public void setGridColumn(Direction gridColumn) {
        this.gridColumn = gridColumn;
    }

    @Override
    public void periodic() {
        if (Settings.isDebug()) {
            Arm.getInstance().getVisualizer().setIntakingPiece(gamePiece);

            Settings.putString("Manager/Game Piece", gamePiece.name());
            Settings.putString("Manager/Node Level", nodeLevel.name());
            Settings.putString("Manager/Score Side", scoreSide.name());
        }
    }
}
