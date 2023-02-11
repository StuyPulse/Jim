package com.stuypulse.robot.subsystems.vision;
import java.util.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.AprilTagData;
import com.stuypulse.robot.util.Limelight;

import static com.stuypulse.robot.constants.Settings.Vision.Limelight.*;
import static com.stuypulse.robot.constants.Settings.Vision.*;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionImpl extends Vision {

    private static final Pose2d kNoPose = 
        new Pose2d(Double.NaN, Double.NaN, Rotation2d.fromDegrees(Double.NaN));

    private final Limelight[] limelights;
    private final List<Result> results;

    private final FieldObject2d[] limelightPoses;

    public VisionImpl() {
        // setup limelight objects
        String[] hostNames = LIMELIGHTS;
        limelights = new Limelight[hostNames.length];

        for(int i = 0; i < hostNames.length; i++){
            limelights[i] = new Limelight(hostNames[i]);
            for (int port : PORTS) {
                PortForwarder.add(port, hostNames[i] + ".local", port);
            }
        }

        // constantly store results in array
        results = new ArrayList<>();

        // store field objects to log poses from limelights
        Field2d field = Odometry.getInstance().getField();
        limelightPoses = new FieldObject2d[limelights.length];
        for (int i = 0; i < limelightPoses.length; ++i) {
            limelightPoses[i] = field.getObject(hostNames[i] + " pose");
        }
    }

    @Override
    public List<Result> getResults() {
        return results;
    }

    // assigns error to data and returns a result 
    private Result process(AprilTagData data) {
        double angleDegrees = absDegToTarget(data.pose, data.id);
        double distance = distanceToTarget(data.pose, data.id);

        SmartDashboard.putNumber("Vision/Angle to Tag", angleDegrees);
        SmartDashboard.putNumber("Vision/Distance", distance);
        SmartDashboard.putNumber("Vision/Tag ID", data.id);

        if (Math.abs(angleDegrees) > TRUST_ANGLE)
            return new Result(data, Noise.HIGH);
        
        boolean inZone = distance <= TRUST_DISTANCE;
        boolean inTolerance = distance <= USABLE_DISTANCE;

        // defaults to high error
        Noise error = Noise.HIGH;

        if(inZone){
            error = Noise.LOW;
        }
        else if (inTolerance) {
            error = Noise.MID;
        }
        return new Result(data, error);
    }


    // helper for process
    private double distanceToTarget(Pose2d pose, int id) {
        if (id < 1)
            return Double.POSITIVE_INFINITY;

        Translation2d robot = pose.getTranslation();
        Translation2d tag = Field.APRIL_TAGS[id-1].toPose2d().getTranslation();
        
        return robot.getDistance(tag);
    }

    private double absDegToTarget(Pose2d pose, int id) {
        if (id < 1)
            return Double.POSITIVE_INFINITY;

        Translation2d robot = pose.getTranslation();
        Translation2d tag = Field.APRIL_TAGS[id-1].toPose2d().getTranslation();

        double deg = robot.minus(tag).getAngle().getDegrees();

        if (Math.abs(deg) > 90)
            deg += 180;

        return deg;
    }


    @Override
    public void periodic(){
        // - clear results array
        // - update cached data in limelights (nicer to do this once per robot loop)
        // - update results array 
        // - log pose onto field and network

        results.clear();
        for (int i = 0; i < limelights.length; ++i) {
            Limelight ll = limelights[i];
            String name = ll.getTableName();
            FieldObject2d pose2d = limelightPoses[i];
            
            ll.updateAprilTagData();
            Optional<AprilTagData> aprilTagData = ll.getAprilTagData();
            
            if (aprilTagData.isPresent()) {
                SmartDashboard.putNumber("Vision/" + name + "/X" , aprilTagData.get().pose.getX());
                SmartDashboard.putNumber("Vision/" + name + "/Y" , aprilTagData.get().pose.getY());
                SmartDashboard.putNumber("Vision/" + name + "/Rotation" , aprilTagData.get().pose.getRotation().getDegrees());
                pose2d.setPose(aprilTagData.get().pose);

                results.add(process(aprilTagData.get()));
            } else {
                SmartDashboard.putNumber("Vision/" + name + "/X" , Double.NaN);
                SmartDashboard.putNumber("Vision/" + name + "/Y" , Double.NaN);
                SmartDashboard.putNumber("Vision/" + name + "/Rotation" , Double.NaN);
                pose2d.setPose(kNoPose);
            }
        }
    }
} 