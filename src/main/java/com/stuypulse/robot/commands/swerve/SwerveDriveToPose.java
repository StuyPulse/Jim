package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveToPose extends CommandBase{
    private final SwerveDrive swerve;
    private final Supplier<Pose2d> targetPoses;

    // Holonomic control
    private final Controller xController;
    private final Controller yController;
    private final AngleController angleController;

    private final BStream aligned;

    private final FieldObject2d targetPose2d;
    
    public SwerveDriveToPose(Supplier<Pose2d> targetPoses){
        this.swerve = SwerveDrive.getInstance();
        this.targetPoses = targetPoses;

        xController = new PIDController(Translation.P,Translation.I,Translation.D);
        yController = new PIDController(Translation.P, Translation.I, Translation.D);
        angleController = new AnglePIDController(Rotation.P, Rotation.I, Rotation.D);
        
        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));

        targetPose2d = Odometry.getInstance().getField().getObject("Target Pose");
        addRequirements(swerve);
    }

    private boolean isAligned() {
        boolean isCone = Manager.getInstance().getGamePiece().isCone();
        return xController.isDone(isCone ? Alignment.ALIGNED_CONE_THRESHOLD_X.get() : Alignment.ALIGNED_CUBE_THRESHOLD_X.get())
            && yController.isDone(isCone ? Alignment.ALIGNED_CONE_THRESHOLD_Y.get() : Alignment.ALIGNED_CUBE_THRESHOLD_Y.get())
            && angleController.isDoneDegrees(isCone ? Alignment.ALIGNED_CONE_THRESHOLD_ANGLE.get() : Alignment.ALIGNED_CUBE_THRESHOLD_ANGLE.get());
    }

    @Override
    public void initialize() {
        Odometry.USE_VISION_ANGLE.set(true);
    }

    @Override
    public void execute() {

        Pose2d currentState = Odometry.getInstance().getPose();
        Pose2d targetPose = targetPoses.get();

        SmartDashboard.putNumber("Alignment/Target Pose X", targetPose.getX());
        SmartDashboard.putNumber("Alignment/Target Pose Y", targetPose.getY());
        SmartDashboard.putNumber("Alignment/Target Pose Rotation (deg)", targetPose.getRotation().getDegrees());


        targetPose2d.setPose(targetPose);

        // boolean alignY = xController.isDone(Units.inchesToMeters(6));

        xController.update(targetPose.getX(), currentState.getX());
        yController.update(targetPose.getY(), currentState.getY());
        angleController.update(Angle.fromRotation2d(targetPose.getRotation()), Angle.fromRotation2d(currentState.getRotation()));

        SmartDashboard.putNumber("Alignment/X Controller Error", xController.getError());
        SmartDashboard.putNumber("Alignment/Y Controller Error", yController.getError());
        SmartDashboard.putNumber("Alignment/Angle Controller Controller Error (deg)", angleController.getError().toDegrees());

        
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            // alignY ? 0 : xController.getOutput(),
            // alignY ? yController.getOutput() : 0,
            xController.getOutput(), yController.getOutput(),
            angleController.getOutput(),
            currentState.getRotation()
        );

        swerve.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public boolean isFinished(){
        return aligned.get();
    }

    public void end(boolean interupted) {
        Odometry.USE_VISION_ANGLE.set(false);
        swerve.stop();
        targetPose2d.setPose(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
    }
    
}
