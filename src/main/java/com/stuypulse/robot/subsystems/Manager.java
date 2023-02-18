package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.util.ArmTrajectory;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmBFSField;
import com.stuypulse.robot.util.ArmState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    public enum Routine {
        INTAKE,
        NEUTRAL,
        READY,
        SCORE,

        BLAY_MODE
    }


    private GamePiece gamePiece;
    private NodeLevel nodeLevel;
    private IntakeSide intakeSide;
    private ScoreSide scoreSide;

    private Direction gridSection;
    private Direction gridColumn;

    private Routine routine;

    public Manager() {
        gamePiece = GamePiece.CUBE;
        nodeLevel = NodeLevel.HIGH;
        intakeSide = IntakeSide.FRONT;
        scoreSide = ScoreSide.SAME;

        routine = Routine.NEUTRAL;

        gridSection = Direction.CENTER;
        gridColumn = Direction.CENTER;
    }

    public ArmBFSField getTrajectory() {
        switch (routine) {
            case INTAKE:
                return getIntakeTrajectory();
            case NEUTRAL:
                return getNeutralTrajectory();
            case READY:
                return getReadyTrajectory();
            case SCORE:
                return getScoreTrajectory();
            default:
                return getNeutralTrajectory(); // TODO: BOOM
        }
    }

    /** Generate Intake Trajectories **/

    private static ArmBFSField.Constraint kIntakeConstraint = (a, w) -> (Math.abs(a + 90) < 30) && (w > 150 || (30 > w));

    private static ArmBFSField kIntakeTrajectory = new ArmBFSField(-55, 0, kIntakeConstraint);

    public ArmBFSField getIntakeTrajectory() {
        if (intakeSide == IntakeSide.FRONT) 
            return kIntakeTrajectory;
        return kIntakeTrajectory.flipped();
    }

    /** Generate Ready Trajectories **/

    /** puts the trajectory on the correct side */
    /** NOTE: trajectory that score "opposite side" must have angles between -90 and -180. **/
    private ArmBFSField normalize(ArmBFSField field) {
        boolean needsFlip = (scoreSide == ScoreSide.SAME && intakeSide == IntakeSide.BACK) ||
            (scoreSide == ScoreSide.OPPOSITE && intakeSide == IntakeSide.FRONT);
        return needsFlip ? field.flipped() : field;
    }
    

    public ArmBFSField getReadyTrajectory() {
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

    private static ArmBFSField kMidReadyTrajectoryCone = new ArmBFSField(-10, 120, kIntakeConstraint);
    private static ArmBFSField kMidReadyTrajectoryCube = new ArmBFSField(-10, 120, kIntakeConstraint);

    private ArmBFSField getMidReadyTrajectory() {
        switch (gamePiece) {
            case CONE_TIP_OUT:
            case CONE_TIP_IN:
                return normalize(kMidReadyTrajectoryCone);

            case CUBE:
                return normalize(kMidReadyTrajectoryCube);

            default:
                return normalize(kNeutralTrajectory);
        }
    }


    private static ArmBFSField kHighReadyTrajectoryCone = new ArmBFSField(10, 120, kIntakeConstraint);
    private static ArmBFSField kHighReadyTrajectoryCube = new ArmBFSField(10, 120, kIntakeConstraint);

    private ArmBFSField getHighReadyTrajectory() {
        switch (gamePiece) {
            case CONE_TIP_IN:
                return normalize(kHighReadyTrajectoryCone);

            case CUBE:
                return normalize(kHighReadyTrajectoryCube);

            default:
                return getNeutralTrajectory();
        }
    }

    /** Generate Score Trajectories **/

    public static ArmBFSField kScoreMidTrajectoryCube = new ArmBFSField(-15, 45, kIntakeConstraint);
    public static ArmBFSField kScoreMidTrajectoryCone = new ArmBFSField(-30, 0, kIntakeConstraint);

    public static ArmBFSField kScoreHighTrajectoryCube = new ArmBFSField(-15, 45, kIntakeConstraint);
    public static ArmBFSField kScoreHighTrajectoryCone = new ArmBFSField(10, -45, kIntakeConstraint);

    public ArmBFSField getScoreTrajectory() {
        switch (nodeLevel) {
            case LOW:
                return getNeutralTrajectory();
            case MID:
                if (gamePiece == GamePiece.CUBE)
                    return normalize(kScoreMidTrajectoryCube);
                
                return normalize(kScoreMidTrajectoryCone);
            case HIGH:
                if (gamePiece == GamePiece.CUBE)
                    return normalize(kScoreHighTrajectoryCube);

                return normalize(kScoreHighTrajectoryCone);
            default:
                return getNeutralTrajectory();
        }
    }

    /** Generate Neutral Trajectories **/

    private static ArmBFSField kNeutralTrajectory = new ArmBFSField(-90, 90, kIntakeConstraint);

    public ArmBFSField getNeutralTrajectory() {
        return kNeutralTrajectory;
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

    private Rotation2d getWestEastAngle(Rotation2d angle) {
        return Math.abs(MathUtil.inputModulus(angle.getDegrees(), -180, 180)) > 90
            ? Rotation2d.fromDegrees(180)
            : Rotation2d.fromDegrees(0);
    }

    private ScoreSide currentScoringSide() {
        Rotation2d normalizedHeading = getWestEastAngle(Odometry.getInstance().getRotation());

        if (normalizedHeading.equals(Rotation2d.fromDegrees(180))) {
            if (intakeSide == IntakeSide.FRONT)
                return ScoreSide.OPPOSITE;
            else
                return ScoreSide.SAME;
        } else {
            if (intakeSide == IntakeSide.FRONT)
                return ScoreSide.SAME;
            else
                return ScoreSide.OPPOSITE;
        }
    }

    private boolean possibleScoringMotion(NodeLevel level, GamePiece piece, ScoreSide side) {
        if (piece == GamePiece.CONE_TIP_OUT) {
            if (level == NodeLevel.HIGH)
                return false;
            
            else if (level == NodeLevel.MID && side == ScoreSide.OPPOSITE)
                return false;
        }
        
        return true;
    }

    public Pose2d getScorePose() {
        ScoreSide side = currentScoringSide();
        Rotation2d currentHeading = SwerveDrive.getInstance().getGyroAngle();
        
        Rotation2d rotation = possibleScoringMotion(nodeLevel, gamePiece, side)
            ? getWestEastAngle(currentHeading)
            : getWestEastAngle(currentHeading).rotateBy(Rotation2d.fromDegrees(180));

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

    public Routine getRoutine() {
        return routine;
    }

    public void setRoutine(Routine routine) {
        this.routine = routine;
    }

    @Override
    public void periodic() {
        Arm.getInstance().getVisualizer().setIntakingPiece(gamePiece);

        SmartDashboard.putString("Manager/Game Piece", gamePiece.name());
        SmartDashboard.putString("Manager/Node Level", nodeLevel.name());
        SmartDashboard.putString("Manager/Intake Side", intakeSide.name());
        SmartDashboard.putString("Manager/Score Side", scoreSide.name());
        SmartDashboard.putString("Manager/Measured Scoring Side", currentScoringSide().name());
    }
}
