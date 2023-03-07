package com.stuypulse.robot.subsystems.odometry;

import java.util.List;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.subsystems.vision.Vision.Noise;
import com.stuypulse.robot.subsystems.vision.Vision.Result;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class OdometryImpl extends Odometry {

    public static final SmartBoolean DISABLE_APRIL_TAGS = new SmartBoolean("Odometry/Disable April Tags", true);

    private interface VisionStdDevs {
        Vector<N3> AUTO_LOW = VecBuilder.fill(10, 10, Math.toRadians(30));
        Vector<N3> AUTO_MID = VecBuilder.fill(15, 15, Math.toRadians(35));

        Vector<N3> TELE_LOW = VecBuilder.fill(3, 3, Math.toRadians(10));
        Vector<N3> TELE_MID = VecBuilder.fill(10, 10, Math.toRadians(15));

        public static Vector<N3> get(Noise noise) {
            if (DriverStation.isAutonomous()) {
                switch (noise) {
                    case LOW: return AUTO_LOW;
                    case MID: return AUTO_MID;
                    default: return AUTO_MID;
                }
            } else {
                switch (noise) {
                    case LOW: return TELE_LOW;
                    case MID: return TELE_MID;
                    default: return null;
                }
            }
        }
    }

    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveOdometry odometry;
    private final Field2d field;

    private final FieldObject2d odometryPose2d;
    private final FieldObject2d poseEstimatorPose2d;

    protected OdometryImpl() {   
        var swerve = SwerveDrive.getInstance();
        var startingPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        
        poseEstimator = 
            new SwerveDrivePoseEstimator(
                swerve.getKinematics(), 
                swerve.getGyroAngle(), 
                swerve.getModulePositions(), 
                startingPose, 

                VecBuilder.fill(
                    Units.inchesToMeters(8), 
                    Units.inchesToMeters(8), 
                    Math.toRadians(2)), 

                    VisionStdDevs.TELE_MID);

        odometry = 
            new SwerveDriveOdometry(
                swerve.getKinematics(), 
                swerve.getGyroAngle(), 
                swerve.getModulePositions(), 
                startingPose);

        field = new Field2d();

        odometryPose2d = field.getObject("Odometry Pose2d");
        poseEstimatorPose2d = field.getObject("Pose Estimator Pose2d");

        swerve.initFieldObjects(field);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void reset(Pose2d pose) {
        SwerveDrive drive = SwerveDrive.getInstance();

        poseEstimator.resetPosition(
            drive.getGyroAngle(), 
            drive.getModulePositions(), 
            pose);

        odometry.resetPosition(
            drive.getGyroAngle(), 
            drive.getModulePositions(), 
            pose);
    }

    @Override
    public Field2d getField() {
        return field;
    }

    private void processResults(List<Result> results, SwerveDrive drive, Vision vision){ 
        if (DISABLE_APRIL_TAGS.get()) {
            return;
        }
        
        for (Result result : results) {
            switch (result.getNoise()) {
                case HIGH:
                    continue;
                case MID:
                    poseEstimator.addVisionMeasurement(
                        result.getPose(),
                        Timer.getFPGATimestamp() - result.getLatency(),
                        VisionStdDevs.get(result.getNoise()));
                    continue;
                default:
                    
                    continue;
            }
        }  
    }

    @Override
    public void periodic() {
        SwerveDrive drive = SwerveDrive.getInstance();
        poseEstimator.update(drive.getGyroAngle(), drive.getModulePositions());
        odometry.update(drive.getGyroAngle(), drive.getModulePositions());

        poseEstimatorPose2d.setPose(poseEstimator.getEstimatedPosition());

        Vision vision = Vision.getInstance();
        List<Result> results = Vision.getInstance().getResults();
        processResults(results, drive, vision);

        odometryPose2d.setPose(odometry.getPoseMeters());
    
        SmartDashboard.putNumber("Odometry/Odometry Pose X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry/Odometry Pose Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry/Odometry Rotation", odometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Odometry/Pose Estimator Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry/Pose Estimator Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Odometry/Pose Estimator Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }
}