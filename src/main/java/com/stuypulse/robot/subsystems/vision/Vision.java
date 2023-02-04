package com.stuypulse.robot.subsystems.vision;
import java.util.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.odometry.IOdometry;
import com.stuypulse.robot.util.AprilTagData;
import com.stuypulse.robot.util.Limelight;

import static com.stuypulse.robot.constants.Field.*;
import static com.stuypulse.robot.constants.Settings.Vision.Limelight.*;
import static com.stuypulse.robot.constants.Settings.Vision.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

public class Vision extends IVision {

    private static final Pose2d kNoPose = 
        new Pose2d(Double.NaN, Double.NaN, Rotation2d.fromDegrees(Double.NaN));

    private final Limelight[] limelights;
    private final List<Result> results;

    private final FieldObject2d[] limelightPoses;

    public Vision() {
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
        Field2d field = IOdometry.getInstance().getField();
        limelightPoses = new FieldObject2d[limelights.length];
        for (int i = 0; i < limelightPoses.length; ++i) {
            limelightPoses[i] = field.getObject(hostNames[i] + " pose");
        }
    }

    @Override
    public List<Result> getResults() {
        return results;
    }

    private Result process(AprilTagData data) {
        double distance = distanceToTarget(data.pose, data.id);
        boolean inZone = distance <= COMMUNITY_DISTANCE;
        boolean inTolerance = distance <= TOLERANCE;

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
        Translation2d robot = pose.getTranslation();
        Translation2d tag = Field.APRIL_TAGS[id-1].toPose2d().getTranslation();
        
        return robot.getDistance(tag);
    }

    @Override
    public void periodic(){
        // - clear results array
        // - update cached data in limelights (nicer to do this once per robot loop)
        // - update results array 
        // - log pose onto field and network

        results.clear();
        for (int i = 0; i < limelights.length; ++i) {
            var ll = limelights[i];
            var pose2d = limelightPoses[i];
            
            ll.updateAprilTagData();
            var aprilTagData = ll.getAprilTagData();
            
            if (aprilTagData.isPresent()) {
                pose2d.setPose(aprilTagData.get().pose);
                results.add(process(aprilTagData.get()));
            } else {
                pose2d.setPose(kNoPose);
            }
        }
    }
}