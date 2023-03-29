package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.ArmTrajectories.*;
import com.stuypulse.robot.constants.Field.ScoreYPoses;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

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

    private GamePiece gamePiece;
    private NodeLevel nodeLevel;
    private ScoreSide scoreSide;

    private SmartNumber gridNode;

    protected Manager() {
        gamePiece = GamePiece.CUBE;
        nodeLevel = NodeLevel.HIGH;
        scoreSide = ScoreSide.FRONT;

        gridNode = new SmartNumber("Manager/Grid Node", 0);
    }

    /** Generate Intake Trajectories **/


    public ArmState getIntakeTrajectory() {
        if (nodeLevel == NodeLevel.HIGH)
            return Acquire.kHPCone;
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

    public int getNearestScoreIndex() {
        var robot = Odometry.getInstance().getTranslation();

        double gridDistance = getSelectedScoreX();
        double[] positions = RobotContainer.getCachedAlliance() == Alliance.Blue ?
            Field.ScoreYPoses.BLUE_Y_POSES : 
            Field.ScoreYPoses.RED_Y_POSES;

        int nearest = 0;
        double nearestDistance = robot.getDistance(new Translation2d(gridDistance, positions[nearest]));

        for (int i = 1; i < positions.length; i++) {
            Translation2d current = new Translation2d(gridDistance, positions[i]);
            double distance = robot.getDistance(current);

            if (distance < nearestDistance) {
                nearest = i;
                nearestDistance = distance;
            }
        }

        return nearest;
    }

    private double getSelectedScoreX() {
        if (nodeLevel == NodeLevel.HIGH) {
            switch (gamePiece) {
                case CUBE:
                    return Field.ScoreXPoses.High.CUBE;
                case CONE_TIP_IN:
                    return Field.ScoreXPoses.High.CONE_TIP_IN;
                case CONE_TIP_OUT:
                    return Field.ScoreXPoses.High.CONE_TIP_OUT;
            }
        } else if (nodeLevel == NodeLevel.MID) {
            switch (gamePiece) {
                case CUBE:
                    return Field.ScoreXPoses.Mid.CUBE;
                case CONE_TIP_IN:
                    return Field.ScoreXPoses.Mid.CONE_TIP_IN;
                case CONE_TIP_OUT:
                    return Field.ScoreXPoses.Mid.CONE_TIP_OUT;
            }
        }
        
        return 0;
    }

    public Translation2d getSelectedScoreTranslation() {
        double gridDistance = getSelectedScoreX();

        if (RobotContainer.getCachedAlliance() == Alliance.Blue) {
            return new Translation2d(
                gridDistance,
                ScoreYPoses.BLUE_Y_POSES[gridNode.intValue()]);
        } else {
            return new Translation2d(
                gridDistance,
                ScoreYPoses.RED_Y_POSES[gridNode.intValue()]);
        }
    }

    public Pose2d getScorePose() {
        Rotation2d rotation = scoreSide == ScoreSide.FRONT ?
            Rotation2d.fromDegrees(180) :
            Rotation2d.fromDegrees(0);

        var translation = getSelectedScoreTranslation();
        SmartDashboard.putNumber("Manager/Selected Score X", translation.getX());
        SmartDashboard.putNumber("Manager/Selected Score Y", translation.getY());
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

    public ScoreSide getScoreSide() {
        return scoreSide;
    }

    public void setScoreSide(ScoreSide scoreSide) {
        this.scoreSide = scoreSide;
    }

    public int getGridNode() {
        return gridNode.intValue();
    }

    public void setGridNode(int gridNode) {
        this.gridNode.set(gridNode);
    }

    @Override
    public void periodic() {
        Arm.getInstance().getVisualizer().setIntakingPiece(gamePiece);

        SmartDashboard.putString("Manager/Game Piece", gamePiece.name());
        SmartDashboard.putString("Manager/Node Level", nodeLevel.name());
        SmartDashboard.putString("Manager/Score Side", scoreSide.name());
    }
}
