package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.odometry.OdometryToggleDataAuton;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.constants.ArmTrajectories;
import com.stuypulse.robot.constants.ArmTrajectories.Ready;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;
import com.stuypulse.robot.util.DebugSequentialCommandGroup;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreePieceWire extends DebugSequentialCommandGroup {

    static class ConeAutonReady extends ArmRoutine {
        public ConeAutonReady() {
            super(Manager.getInstance()::getReadyTrajectory);
        }

        @Override
        protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
    
            return new ArmTrajectory()
                .addState(new ArmState(dest.getShoulderState(), src.getWristState())
                            .setShoulderTolerance(20).setWristLimp(true).setWristTolerance(360))
                .addState(new ArmState(dest.getShoulderState(), dest.getWristState()).setWristTolerance(7)
                            .setShoulderTolerance(15));
        }
    }

    static class AutonReady extends ArmRoutine {
        public AutonReady() {
            super(() -> {
                if (Manager.getInstance().getNodeLevel() == NodeLevel.HIGH) {
                    return Ready.High.kCubeAutonBack;
                } else {
                    return Ready.Mid.kAutonCubeBack;
                }
            });
        }

        @Override
        protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
            double wristSafeAngle = Wrist.WRIST_SAFE_ANGLE.get();
    
            return new ArmTrajectory()
                .addState(new ArmState(src.getShoulderDegrees(), wristSafeAngle)
                    .setWristTolerance(45))
                .addState(new ArmState(dest.getShoulderState(), dest.getWristState())
                    .setWristTolerance(23).setShoulderTolerance(20));
        }
    }

    static class ArmIntakeFirst extends ArmRoutine {
        public ArmIntakeFirst() {
            super(() -> ArmTrajectories.Acquire.kBOOMCubeAuton);
        }

        @Override
        public void initialize() {
            super.initialize();
            Wrist.SHOULDER_VELOCITY_FEEDBACK_CUTOFF.set(5.0);
        }

        @Override
        protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
            dest = new ArmState(
                -70.82,
                11);
               // 8.37);
    
            return new ArmTrajectory()
                // .addState(src.getShoulderDegrees(), wristSafeAngle)
    
                // .addState(
                //     new ArmState(intermediateShoulderDegrees, wristSafeAngle)
                //         .setShoulderTolerance(15)
                //         .setWristTolerance(360))
    
                // .addState(
                //     new ArmState(intermediateShoulderDegrees, dest.getWristDegrees())
                //         .setWristTolerance(360))
    
                .addState(
                    new ArmState(dest.getShoulderDegrees(), dest.getWristDegrees())
                        .setShoulderTolerance(3)
                        .setWristTolerance(4));
        }

        @Override
        public void end(boolean i) {
            super.end(i);
            Wrist.SHOULDER_VELOCITY_FEEDBACK_CUTOFF.set(10.0);
        }
    }

    static class ArmIntakeSecond extends ArmRoutine {
        public ArmIntakeSecond() {
            super(() -> ArmTrajectories.Acquire.kBOOMCubeAuton);
        }

        @Override
        public void initialize() {
            super.initialize();
            Wrist.SHOULDER_VELOCITY_FEEDBACK_CUTOFF.set(5.0);
        }

        @Override
        protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
            dest = new ArmState(
                -70.82,
                10);
               // 8.37);
            double intermediateShoulderDegrees = Manager.getInstance().getIntakeIntermediateTrajectory().getShoulderDegrees();
            double wristSafeAngle = Wrist.WRIST_SAFE_ANGLE.get();
    
            return new ArmTrajectory()
                // .addState(src.getShoulderDegrees(), wristSafeAngle)
    
                .addState(
                    new ArmState(intermediateShoulderDegrees, wristSafeAngle)
                        .setShoulderTolerance(15)
                        .setWristTolerance(15))
    
                .addState(
                    new ArmState(intermediateShoulderDegrees, dest.getWristDegrees())
                        .setWristTolerance(20))
    
                .addState(
                    new ArmState(dest.getShoulderDegrees(), dest.getWristDegrees())
                        .setShoulderTolerance(3)
                        .setWristTolerance(4));
        }

        @Override
        public void end(boolean i) {
            super.end(i);

            Wrist.SHOULDER_VELOCITY_FEEDBACK_CUTOFF.set(10.0);
        }
    }

    private static final double INTAKE_DEACQUIRE_TIME = 0.5;
    private static final double INTAKE_STOP_WAIT_TIME = 0.5;
    private static final double INTAKE_WAIT_TIME = 1.0;
    private static final double ACQUIRE_WAIT_TIME = 0.3;
    private static final double WIGGLE_PERIOD = 0.6;
    private static final double WIGGLE_VEL_AMPLITUDE = 0.6;

    private static final double ALIGNMENT_TIME = 2;

    private static final PathConstraints INTAKE_SECOND_PIECE_CONSTRAINTS = new PathConstraints(2, 2);
    private static final PathConstraints INTAKE_THIRD_PIECE_CONSTRAINTS = new PathConstraints(2.2, 1.5);

    private static final PathConstraints SCORE_PIECE_CONSTRAINTS = new PathConstraints(4.5, 3.5);
    private static final PathConstraints THIRD_SCORE_PIECE_CONSTRAINTS = new PathConstraints(4.5, 3.5);

    public ThreePieceWire() {

        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("3 Piece Wire FAST", INTAKE_SECOND_PIECE_CONSTRAINTS, SCORE_PIECE_CONSTRAINTS, INTAKE_THIRD_PIECE_CONSTRAINTS, THIRD_SCORE_PIECE_CONSTRAINTS),
            "Intake Piece", "Score Piece", "Intake Third Piece", "Score Third Piece"
        );

        var arm = Arm.getInstance();
        
        // initial setup
        addCommands(
            // new OdometryToggleDataAuton(true),
            new ManagerSetNodeLevel(NodeLevel.MID),
            new ManagerSetGamePiece(GamePiece.CONE_TIP_UP),
            new ManagerSetScoreSide(ScoreSide.BACK)
        );

        // score first piece
        addCommands(
            new LEDSet(LEDColor.RED),
            new ConeAutonReady()
                .withTimeout(1.5)
        );

        addCommands(
            new LEDSet(LEDColor.BLUE),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        // intake second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),

            new LEDSet(LEDColor.GREEN),

            new ParallelDeadlineGroup(
                new SwerveDriveFollowTrajectory(paths.get("Intake Piece"))
                    .robotRelative()
                    .withStop()
                    .withStallStop(),
                    // .until(Intake.getInstance()::hasGamePiece), // interrupting IntakeScore? idk one time the intake just stopped early

                new WaitCommand(INTAKE_STOP_WAIT_TIME)
                    .andThen(new IntakeStop())
                    .andThen(new IntakeAcquire()),

                new ArmIntakeFirst()
                    .withTolerance(4, 10)
            ),

            new WaitCommand(ACQUIRE_WAIT_TIME).until(Intake.getInstance()::hasGamePiece)
                .alongWith(arm.runOnce(() -> arm.setWristVoltage(-3))),

            // new SwerveDriveWiggle(WIGGLE_PERIOD, WIGGLE_VEL_AMPLITUDE)
            //     .until(Intake.getInstance()::hasGamePiece)
            //     .alongWith(arm.runOnce(() -> arm.setWristVoltage(-3)))
            //     .withTimeout(ACQUIRE_WAIT_TIME),

            arm.runOnce(() -> arm.setWristVoltage(0))
        );

        // drive to grid and score second piece :: TODO: make custom arm setpoint for this
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),
            new ManagerSetScoreSide(ScoreSide.BACK),

            new LEDSet(LEDColor.RED),

            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece"))
                    .fieldRelative()
                .withStop()
                .alongWith(new AutonReady()
                .alongWith(new WaitCommand(0.5).andThen(new IntakeStop()))),

            new ManagerSetGridNode(7),
            // new SwerveDriveToScorePose().withTimeout(ALIGNMENT_TIME),
            new IntakeDeacquire(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop()
        );

        // intake third piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),

            new LEDSet(LEDColor.GREEN),

            new ParallelDeadlineGroup(
                new SwerveDriveFollowTrajectory(paths.get("Intake Third Piece"))
                        .fieldRelative()
                        .withStallStop()
                        .until(Intake.getInstance()::hasGamePiece),

                new WaitCommand(INTAKE_STOP_WAIT_TIME)
                    .andThen(new IntakeStop())
                    .andThen(new WaitCommand(INTAKE_WAIT_TIME))
                    .andThen(new IntakeAcquire()),

                new ArmIntakeSecond()
                    // .setShoulderVelocityTolerance(10)
                    // .withTolerance(4, 10)
            ),

            new SwerveDriveWiggle(WIGGLE_PERIOD, WIGGLE_VEL_AMPLITUDE)
                .until(Intake.getInstance()::hasGamePiece)
                .alongWith(arm.runOnce(() -> arm.setWristVoltage(-3)))
                .withTimeout(ACQUIRE_WAIT_TIME),

            arm.runOnce(() -> arm.setWristVoltage(0))
        );

        // drive to grid and score third piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),
            new ManagerSetNodeLevel(NodeLevel.HIGH),
            new ManagerSetScoreSide(ScoreSide.BACK),

            new LEDSet(LEDColor.RED),

            new SwerveDriveFollowTrajectory(
                paths.get("Score Third Piece"))
                    .fieldRelative()
                .withStop()
                .alongWith(new AutonReady().alongWith(new WaitCommand(0.5).andThen(new IntakeStop()))),

            new ManagerSetGridNode(7),
            // new SwerveDriveToScorePose().withTimeout(ALIGNMENT_TIME),
            new IntakeDeacquire(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop()
        );

        // back away
        addCommands(
            new LEDSet(LEDColor.RAINBOW)

            // new SwerveDriveFollowTrajectory(
            //     paths.get("Back Away"))
            //         .fieldRelative()
            //     .withStop()
        );
    }
}
