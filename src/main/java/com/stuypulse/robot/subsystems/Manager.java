package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.ArmTrajectories.*;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manager extends SubsystemBase {

    // singleton
    private static Manager instance;

    static {
        instance = new Manager();
    }

    public static Manager getInstance() {
        return instance;
    }

    // game piece to score
    public enum GamePiece {
        CONE_TIP_IN(false),
        CONE_TIP_UP(false),
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

    private SmartBoolean scoreFront = new SmartBoolean("Manager/Score Front", false);

    protected Manager() {
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
        return getLowReadyTrajectory();
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
                return getStowTrajectory();
        }
    }

    private ArmState getLowReadyTrajectory() {
        return scoreSide == ScoreSide.FRONT ? Deacquire.kFrontTrajectory : Deacquire.kBackTrajectory;
    }

    private ArmState getMidReadyTrajectory() {
        switch (gamePiece) {
            case CONE_TIP_OUT:
                return Ready.Mid.kConeTipOutFront;

            case CONE_TIP_IN:
                return Ready.Mid.kConeTipInBack;

            case CUBE:
                return scoreSide == ScoreSide.FRONT ? Ready.Mid.kCubeFront : Ready.Mid.kCubeBack;

            default:
                return getStowTrajectory();
        }
    }

    private ArmState getHighReadyTrajectory() {
        switch (gamePiece) {
            case CONE_TIP_IN:
                return Ready.High.kConeTipInBack;
            
            case CONE_TIP_OUT:
                return Ready.High.kConeTipOutFront;

            case CONE_TIP_UP:
                return Ready.High.kConeTipUpBack;
                
            case CUBE:
                return scoreSide == ScoreSide.FRONT ? Ready.High.kCubeFront : Ready.High.kCubeBack;

            default:
                return getStowTrajectory();
        }
    }

    /** Generate Neutral Trajectories **/

    // wrist faces away from scoring direction for cube
    public ArmState getStowTrajectory() {
        return Stow.kTrajectory;
    }

    /** Generate Score Pose **/

    public Translation2d getNearestScoreTranslation() {
        var robot = Odometry.getInstance().getTranslation();

        Translation2d[] positions = RobotContainer.getCachedAlliance() == Alliance.Blue ?
            Field.BLUE_ALIGN_POSES : 
            Field.RED_ALIGN_POSES;

        Translation2d nearest = positions[0];
        double nearestDistance = robot.getDistance(nearest);

        for (int i = 1; i < positions.length; i++) {
            double distance = robot.getDistance(positions[i]);

            if (distance < nearestDistance) {
                nearest = positions[i];
                nearestDistance = distance;
            }
        }

        return nearest;
    }

    public Translation2d getSelectedScoreTranslation() {
        int index = gridSection.ordinal() * 3 + gridColumn.ordinal();
        
        if (RobotContainer.getCachedAlliance() == Alliance.Blue) {
            return Field.BLUE_ALIGN_POSES[index];
        } else {
            return Field.RED_ALIGN_POSES[index];
        }
    }

    public Pose2d getScorePose() {
        Rotation2d rotation = scoreSide == ScoreSide.FRONT ?
            Rotation2d.fromDegrees(180) :
            Rotation2d.fromDegrees(0);

        return new Pose2d(getSelectedScoreTranslation(), rotation);
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
        Arm.getInstance().getVisualizer().setIntakingPiece(gamePiece);

        setScoreSide(scoreFront.get() ? ScoreSide.FRONT : ScoreSide.BACK);

        SmartDashboard.putString("Manager/Game Piece", gamePiece.name());
        SmartDashboard.putString("Manager/Node Level", nodeLevel.name());
        SmartDashboard.putString("Manager/Score Side", scoreSide.name());
        SmartDashboard.putString("Manager/Grid Section", gridSection.name());
        SmartDashboard.putString("Manager/Grid Column", gridColumn.name());
    }
}
