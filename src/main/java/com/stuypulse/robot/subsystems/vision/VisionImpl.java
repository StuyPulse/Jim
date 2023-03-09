package com.stuypulse.robot.subsystems.vision;
import java.util.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.AprilTagData;
import com.stuypulse.robot.util.Limelight;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import static com.stuypulse.robot.constants.Settings.Vision.Limelight.*;
import static com.stuypulse.robot.constants.Settings.Vision.*;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionImpl extends Vision {

    private static final Pose2d kNoPose = 
        new Pose2d(Double.NaN, Double.NaN, Rotation2d.fromDegrees(Double.NaN));

    private static final AprilTagData kNoData =
        new AprilTagData(kNoPose, Double.NaN, -1);

    // store limelight network tables
    private final Limelight[] limelights;

    // store fieldobject2d's to display limelight data
    private final FieldObject2d[] limelightPoses;

    // cache results every loop in a list
    private final List<AprilTagData> results;

    protected VisionImpl() {
        // reference to all limelights on robot
        String[] hostNames = LIMELIGHTS;

        // setup limelight objects and field objects for april tag data
        limelights = new Limelight[hostNames.length];
        limelightPoses = new FieldObject2d[limelights.length];

        // setup all objects
        Field2d field = Odometry.getInstance().getField();
        for(int i = 0; i < hostNames.length; i++){
            limelights[i] = new Limelight(hostNames[i]);
            limelightPoses[i] = field.getObject(hostNames[i] + " pose");

            for (int port : PORTS) {
                PortForwarder.add(port, hostNames[i] + ".local", port);
            }
        }

        // constantly store results in array
        results = new ArrayList<>();
    }

    @Override
    public List<AprilTagData> getResults() {
        return results;
    }

    // helper for process
    private static double getDistanceToTag(Pose2d pose, int id) {
        if (id < 1)
            return Double.POSITIVE_INFINITY;

        Translation2d robot = pose.getTranslation();
        Translation2d tag = Field.getAprilTagFromId(id).toPose2d().getTranslation();
        
        return robot.getDistance(tag);
    }

    private static double getDegreesToTag(Pose2d pose, int id) {
        if (id < 1)
            return Double.POSITIVE_INFINITY;

        Rotation2d tag = Field.getAprilTagFromId(id).toPose2d().getRotation();

        return getDegreesBetween(tag.plus(Rotation2d.fromDegrees(180)), pose.getRotation());
    }

    private static void putAprilTagData(String prefix, AprilTagData data) {
        SmartDashboard.putNumber(prefix + "/Pose X" , data.pose.getX());
        SmartDashboard.putNumber(prefix + "/Pose Y" , data.pose.getY());
        SmartDashboard.putNumber(prefix + "/Pose Rotation (Deg)" , data.pose.getRotation().getDegrees());
        SmartDashboard.putNumber(prefix + "/Tag ID", data.id);
        SmartDashboard.putNumber(prefix + "/Latencty (s)", data.latency);
    }

    private static double getDegreesBetween(Rotation2d a, Rotation2d b) {
        double c = a.getCos() * b.getCos() + a.getSin() * b.getSin();
        double d = (1 - c) * 180;

        return d;
    }

    private static boolean isAcceptable(Pose2d robot, Pose2d vision, int tagid) {
        // check if distance to tag is greater than cutoff
        double distanceToTag = getDistanceToTag(vision, tagid);
        if (distanceToTag < MIN_USE_DISTANCE || distanceToTag > MAX_USE_DISTANCE) return false;

        // check if angle to tag is greater than cutoff
        double angleToTag = Math.abs(getDegreesToTag(vision, tagid));
        if (angleToTag < MAX_USE_ANGLE || angleToTag > MAX_USE_ANGLE) return false;

        // OK
        return true;
    }

    @Override
    public void periodic(){
        // - clear results array
        // - update cached data in limelights (nicer to do this once per robot loop)
        // - update results array 
        // - log pose onto field and network

        var robotPose = Odometry.getInstance().getPose();
        // TODO: log all vision measurements, but also indicate which ones were actually used


        results.clear();
        for (int i = 0; i < limelights.length; ++i) {
            Limelight ll = limelights[i];
            FieldObject2d ll2d = limelightPoses[i];
            
            ll.updateAprilTagData();

            if (ll.hasAprilTagData()) {
                var data = ll.getAprilTagData().get();

                if (isAcceptable(robotPose, data.pose, data.id)) {
                    if (!Field.isValidAprilTagId(data.id)) continue;
                    putAprilTagData("Vision/" + ll.getTableName(), data);
                    ll2d.setPose(data.pose);
                    
                    results.add(data);
                } else {
                    putAprilTagData("Vision/" + ll.getTableName(), kNoData);
                    ll2d.setPose(kNoPose);
                }


            } else {
                putAprilTagData("Vision/" + ll.getTableName(), kNoData);
                ll2d.setPose(kNoPose);
            }
        }
    }
} 