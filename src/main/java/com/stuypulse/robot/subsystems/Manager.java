package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.util.ArmTrajectory;
import com.stuypulse.robot.util.AStar.*;
import com.stuypulse.robot.util.AStar.AstarUtil;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        CONE_TIP_IN(false),
        CONE_TIP_OUT(false),
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

    public enum Direction {
        LEFT,
        CENTER,
        RIGHT
    }


    private GamePiece gamePiece;
    private NodeLevel nodeLevel;
    private IntakeSide intakeSide;
    private ScoreSide scoreSide;

    private Direction gridSection;
    private Direction gridColumn;

    public Manager() {
        gamePiece = GamePiece.CUBE;
        nodeLevel = NodeLevel.HIGH;
        intakeSide = IntakeSide.FRONT;
        scoreSide = ScoreSide.SAME;

        gridSection = Direction.CENTER;
        gridColumn = Direction.CENTER;
    }

    /** Generate Intake Trajectories **/

    /** puts the trajectory on the correct side */
    /** NOTE: trajectory that score "opposite side" must have angles between -90 and -180. **/
    private ArmTrajectory normalize(ArmTrajectory trajectory) {
        boolean needsFlip = (scoreSide == ScoreSide.SAME && intakeSide == IntakeSide.BACK) ||
            (scoreSide == ScoreSide.OPPOSITE && intakeSide == IntakeSide.FRONT);

        return needsFlip ? trajectory.flipped() : trajectory;
    }
    
    public ArmTrajectory getNeutralTrajectory() {
        return AstarUtil.generateTrajectory(
                (s, w) -> false,
                Arm.getInstance().getState(), 
                ArmState.fromDegrees(-90, 90));
    }

    public ArmTrajectory getIntakeTrajectory() {
        System.out.println("TRYING TO GENERATE INTAKE TRAJECTORY");
        ArmTrajectory intakeTrajectory = AstarUtil.generateTrajectory(
            (s, w) -> false,
            Arm.getInstance().getState(),
            IntakeSide.FRONT == intakeSide ? 
                ArmState.fromDegrees(-55, 0) :
                ArmState.fromDegrees(-180 - (-55), -180 - 0));
        System.out.println("GENERATED INTAKE TRAJECTORY");

        System.out.println("GET TOTAL TIME: " + intakeTrajectory.getTotalTime());
        return intakeTrajectory;
        // return intakeSide == IntakeSide.FRONT ? intakeTrajectory : intakeTrajectory.flipped();
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

    /** Generate Score Trajectories **/

    public ArmTrajectory getScoreTrajectory() {
        switch (nodeLevel) {
            case LOW:
                return getIntakeTrajectory();
            case MID:
                if (gamePiece == GamePiece.CUBE)
                    return normalize(new ArmTrajectory().addState(ArmState.fromDegrees(-15, 45)));
                
                return normalize(ArmTrajectory.fromStates(
                    ArmState.fromDegrees(-30, 0)));
            case HIGH:
                if (gamePiece == GamePiece.CUBE)
                    return normalize(new ArmTrajectory().addState(ArmState.fromDegrees(-15, 45)));

                return normalize(ArmTrajectory.fromStates(
                    ArmState.fromDegrees(10, -45)));
            default:
                return getNeutralTrajectory();
        }
    }

    /** Generate Neutral Trajectories **/

    private ArmTrajectory getMidReadyTrajectory() {
        switch (gamePiece) {
            case CONE_TIP_IN:
                return normalize(AstarUtil.generateTrajectory(
                    (s, w) -> false,
                    new ArmState[] { 
                        Arm.getInstance().getState(), 
                        ArmState.fromDegrees(-5, -136)
                    }));
            case CONE_TIP_OUT:
                return normalize(AstarUtil.generateTrajectory(
                    (s, w) -> false,    
                    new ArmState[] { 
                        Arm.getInstance().getState(), 
                        ArmState.fromDegrees(-37, 87)
                    }));
            case CUBE:
                return normalize(AstarUtil.generateTrajectory(
                    (s, w) -> false,    
                new ArmState[] { 
                        Arm.getInstance().getState(), 
                        ArmState.fromDegrees(-5, -136)}));
            default:
                return getNeutralTrajectory();
        }
    }

    private ArmTrajectory getHighReadyTrajectory() {
        switch (gamePiece) {
            case CONE_TIP_IN:
                return normalize(AstarUtil.generateTrajectory(
                    (s, w) -> false,
                    new ArmState[] { 
                        Arm.getInstance().getState(), 
                        ArmState.fromDegrees(-8, -32)}));

            case CONE_TIP_OUT:
                return getNeutralTrajectory();

            case CUBE:
                return normalize(AstarUtil.generateTrajectory(
                    (s, w) -> false,
                    new ArmState[] { 
                        Arm.getInstance().getState(), 
                        ArmState.fromDegrees(-8, -32)}));

            default:
                return getNeutralTrajectory();
        }
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
        var translation = getScoreTranslation();
        var rotation = new Rotation2d(); 

        return new Pose2d(translation, rotation);
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
        Arm.getInstance().getVisualizer().setIntakingPiece(gamePiece);

        SmartDashboard.putString("Manager/Game Piece", gamePiece.name());
        SmartDashboard.putString("Manager/Node Level", nodeLevel.name());
        SmartDashboard.putString("Manager/Intake Side", intakeSide.name());
        SmartDashboard.putString("Manager/Score Side", scoreSide.name());
    }
}
