package com.stuypulse.robot.subsystems.vision;
import java.util.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.AprilTagData;
import com.stuypulse.robot.util.Limelight;

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
    private final List<Result> results;

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
    public List<Result> getResults() {
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

        Translation2d robot = pose.getTranslation();
        Translation2d tag = Field.getAprilTagFromId(id).toPose2d().getTranslation();
        
        double deg = robot.minus(tag).getAngle().getDegrees();
        if (Math.abs(deg) > 90)
            deg += 180;

        return deg;
    }

    // assigns error to data and returns a result 
    private Result process(AprilTagData data) {
        if (!Field.isValidAprilTagId(data.id))
            return new Result(data, Noise.HIGH);

        double angleDegrees = getDistanceToTag(data.pose, data.id);
        double distance = getDegreesToTag(data.pose, data.id);

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

    private static boolean isAcceptable(Pose2d robot, Pose2d vision) {
        // check if distance greater than cutoff
        double distance = robot.getTranslation().getDistance(vision.getTranslation());
        if (distance > Units.feetToMeters(4)) return false;

        // check if angle is greater than cutoff
        double degrees = getDegreesBetween(robot.getRotation(), vision.getRotation());
        if (degrees > 6) return false;

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

                if (isAcceptable(robotPose, data.pose)) {
                    putAprilTagData("Vision/" + ll.getTableName(), data);
                    ll2d.setPose(data.pose);
                    
                    results.add(new Result(data, Noise.LOW));
                } else {
                    putAprilTagData("Vision/" + ll.getTableName(), kNoData);
                    ll2d.setPose(kNoPose);    
                
                    System.out.println("Rejected data from " + ll.getTableName());
                }


            } else {
                putAprilTagData("Vision/" + ll.getTableName(), kNoData);
                ll2d.setPose(kNoPose);
            }
        }
    }
} 