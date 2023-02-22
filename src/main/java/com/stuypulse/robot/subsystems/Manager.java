package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.ArmFields.Intake;
import com.stuypulse.robot.constants.ArmFields.Neutral;
import com.stuypulse.robot.constants.ArmFields.Outtake;
import com.stuypulse.robot.constants.ArmFields.Ready;
import com.stuypulse.robot.constants.ArmFields.Score;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.util.ArmBFSField;

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
        BACK;

        public IntakeSide getOpposite() {
            return (this == FRONT) ? BACK : FRONT;
        }
    }

    // side to score on (relative to intake side)
    public enum ScoreSide {
        SAME,
        OPPOSITE;

        public ScoreSide getOpposite() {
            return (this == SAME) ? OPPOSITE : SAME;
        }
    }

    // Direction that describes a scoring position
    public enum Direction {
        LEFT,
        CENTER,
        RIGHT
    }

    // Routine for the arm 
    public enum Routine {
        INTAKE,
        OUTTAKE,
        NEUTRAL,
        READY,
        SCORE,

        MANUAL_CONTROL
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

        // Doesn't matter what this starts at because
        // the arm doesn't start with a trajectory.
        routine = Routine.MANUAL_CONTROL;

        gridSection = Direction.CENTER;
        gridColumn = Direction.CENTER;
    }

    public ArmBFSField getTrajectory() {
        switch (routine) {
            case INTAKE:
                return getIntakeTrajectory();
            case OUTTAKE:
                return getOuttakeTrajectory();
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


    public ArmBFSField getIntakeTrajectory() {
        if (intakeSide == IntakeSide.FRONT) 
            return Acquire.kTrajectory;
        return Acquire.kTrajectory.flipped();
    }

    public ArmBFSField getOuttakeTrajectory() {
        if (intakeSide == IntakeSide.FRONT) 
            return Deacquire.kTrajectory;
        return Deacquire.kTrajectory.flipped();
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
                return getLowReadyTrajectory();

            case MID:
                return getMidReadyTrajectory();

            case HIGH:
                return getHighReadyTrajectory();

            default:
                return getNeutralTrajectory();
        }
    }

    private ArmBFSField getLowReadyTrajectory() {
        switch (gamePiece) {
            case CONE_TIP_OUT:
                if (scoreSide == ScoreSide.SAME)
                    return normalize(Ready.Low.kConeTipOutSame);
                return normalize(Ready.Low.kConeTipOutOpposite);
            
            case CONE_TIP_IN:
                if (scoreSide == ScoreSide.SAME)
                    return normalize(Ready.Low.kConeTipInSame);
                return normalize(Ready.Low.kConeTipInOpposite);
            
            case CUBE:
                return normalize(Ready.Low.kCube);
            
            default:
                return Neutral.kTrajectory;
        }
    }

    private ArmBFSField getMidReadyTrajectory() {
        switch (gamePiece) {
            case CONE_TIP_OUT:
                // impossible to score tip out opposite side on mid
                return normalize(Ready.Mid.kConeTipOutSame);

            case CONE_TIP_IN:
                return normalize(Ready.Mid.kConeTipInOpposite);

            case CUBE:
                return normalize(Ready.Mid.kCube);

            default:
                return getNeutralTrajectory();
        }
    }

    private ArmBFSField getHighReadyTrajectory() {
        switch (gamePiece) {
            case CONE_TIP_IN:
                return normalize(Ready.High.kConeTipInOpposite);

            case CUBE:
                return normalize(Ready.High.kCube);

            default:
                return getNeutralTrajectory();
        }
    }

    /** Generate Score Trajectories **/

    public ArmBFSField getScoreTrajectory() {
        switch (nodeLevel) {
            case LOW:
                return getLowReadyTrajectory();

            case MID:
                if (gamePiece == GamePiece.CUBE)
                    return normalize(Score.Mid.kCube);
                
                if (gamePiece == GamePiece.CONE_TIP_OUT)
                    return normalize(Score.Mid.kConeTipOutSame);

                return normalize(Score.Mid.kConeTipInOpposite);

            case HIGH:
                if (gamePiece == GamePiece.CUBE)
                    return normalize(Score.High.kCube);

                return normalize(Score.High.kConeTipInOpposite);

            default:
                return getNeutralTrajectory();
        }
    }

    /** Generate Neutral Trajectories **/

    // wrist faces away from scoring direction for cube
    public ArmBFSField getNeutralTrajectory() {
        if (Intake.getInstance().getIntookSide() == IntakeSide.FRONT)
            return Neutral.kTrajectory;
        return Neutral.kTrajectory.flipped();
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

        if (intakeSide == IntakeSide.FRONT && scoreSide == ScoreSide.SAME)
            rotation = Rotation2d.fromDegrees(180);
        else if (intakeSide == IntakeSide.BACK && scoreSide == ScoreSide.OPPOSITE)
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
        if (Settings.isDebug()) {
            Arm.getInstance().getVisualizer().setIntakingPiece(gamePiece);

            Settings.putString("Manager/Game Piece", gamePiece.name());
            Settings.putString("Manager/Node Level", nodeLevel.name());
            Settings.putString("Manager/Intake Side", intakeSide.name());
            Settings.putString("Manager/Score Side", scoreSide.name());
            Settings.putString("Manager/Routine", routine.name());
        }
    }
}
