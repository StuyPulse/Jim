/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.vision;
import static com.stuypulse.robot.constants.Settings.Vision.*;
import static com.stuypulse.robot.constants.Settings.Vision.Limelight.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.AprilTagData;
import com.stuypulse.robot.util.Limelight;
import com.stuypulse.robot.util.Limelight.DataType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;

public class VisionImpl extends Vision {

    public enum DataStatus {
        ACCEPTED,
        REJECTED,
        NONE
    }

    private static final Pose2d kNoPose =
        new Pose2d(Double.NaN, Double.NaN, Rotation2d.fromDegrees(Double.NaN));

    private static final AprilTagData kNoData =
        new AprilTagData(kNoPose, Double.NaN, -1, null);

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
            limelights[i] = new Limelight(hostNames[i], POSITIONS[i]);
            limelightPoses[i] = field.getObject(hostNames[i] + " pose");

            for (int port : PORTS) {
                PortForwarder.add(port + i * 10, hostNames[i] + ".local", port);
            }
        }

        // constantly store results in array
        results = new ArrayList<>();
    }

    @Override
    public void setPipeline(DataType type) {
        for (Limelight limelight : limelights) {
            limelight.setPipeline(type);
        }
    }

    @Override
    public DataType getPipeline() {
        return limelights[0].getPipeline();
    }

    @Override
    public double getDistance() {
        for (Limelight limelight : limelights) {
            if (limelight.hasReflectiveTapeData()) {
                return limelight.getDistance();
            }
        }
        return Double.NaN;
    }

    @Override
    public double getAngle() {
        for (Limelight limelight : limelights) {
            if (limelight.hasReflectiveTapeData()) {
                return limelight.getXAngle();
            }
        }
        return Double.NaN;
    }

    @Override
    public List<AprilTagData> getResults() {
        return results;
    }

    private static void putAprilTagData(String prefix, AprilTagData data, DataStatus accepted) {
        SmartDashboard.putNumber(prefix + "/Pose X" , data.pose.getX());
        SmartDashboard.putNumber(prefix + "/Pose Y" , data.pose.getY());
        SmartDashboard.putNumber(prefix + "/Pose Rotation (Deg)" , data.pose.getRotation().getDegrees());
        SmartDashboard.putNumber(prefix + "/Tag ID", data.id);
        SmartDashboard.putNumber(prefix + "/Latencty (s)", data.latency);

        if (accepted != DataStatus.NONE) {
            SmartDashboard.putNumber(prefix + "/Distance to Tag", data.getDistanceToTag());
            SmartDashboard.putNumber(prefix + "/Angle to Tag", data.getDegreesToTag());
        } else {
            SmartDashboard.putNumber(prefix + "/Distance to Tag", Double.NaN);
            SmartDashboard.putNumber(prefix + "/Angle to Tag", Double.NaN);
        }

        SmartDashboard.putString(prefix + "/Accepted", accepted.name());
    }

    private static boolean isAcceptable(Pose2d robot, AprilTagData data) {
        // reject invalid apriltag ids
        if (!Field.isValidAprilTagId(data.id)) return false;

        // check if distance to tag is greater than cutoff
        double distanceToTag = data.getDistanceToTag();
        if (distanceToTag < MIN_USE_DISTANCE || distanceToTag > MAX_USE_DISTANCE) return false;

        // check if angle to tag is greater than cutoff
        double angleToTag = Math.abs(data.getDegreesToTag());
        if (angleToTag < MIN_USE_ANGLE || angleToTag > MAX_USE_ANGLE) return false;

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

                Odometry.getInstance().getField().getObject("April Tag").setPose(Field.getAprilTagFromId(data.id));

                if (isAcceptable(robotPose, data)) {
                    if (!Field.isValidAprilTagId(data.id)) continue;
                    putAprilTagData("Vision/" + ll.getTableName(), data, DataStatus.ACCEPTED);
                    ll2d.setPose(data.pose);

                    results.add(data);
                } else {
                    putAprilTagData("Vision/" + ll.getTableName(), data, DataStatus.REJECTED);
                    ll2d.setPose(kNoPose);
                }

            } else {
                putAprilTagData("Vision/" + ll.getTableName(), kNoData, DataStatus.NONE);
                ll2d.setPose(kNoPose);
            }
        }
    }
}