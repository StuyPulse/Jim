package com.stuypulse.robot.subsystems.vision;
import java.util.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.util.AprilTagData;
import com.stuypulse.robot.util.Limelight;

import static com.stuypulse.robot.constants.Field.*;
import static com.stuypulse.robot.constants.Settings.Vision.Limelight.*;
import static com.stuypulse.robot.constants.Settings.Vision.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends IVision {

    private final Limelight[] limelights;
    private final List<Result> results;

    public Vision() {
        String[] hostNames = LIMELIGHTS;
        limelights = new Limelight[hostNames.length];

        for(int i = 0; i < hostNames.length; i++){
            limelights[i] = new Limelight(hostNames[i]);
            for (int port : PORTS) {
                PortForwarder.add(port, hostNames[i] + ".local", port);
            }
        }

        results = new ArrayList<>();
    }

    public List<Result> getResults() {
        results.clear();

        for (Limelight ll : limelights){
            Optional<AprilTagData> data = ll.getPoseData();

            if(data.isPresent()) {
                results.add(process(data.get(),ll));
            }
        }
        return results;
    }

    private Result process(AprilTagData data, Limelight limelight) {
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

        return new Result(data, error, limelight.getTableName());
    }


    // helper for process
    private double distanceToTarget(Pose2d pose, int id) {
        Translation2d robot = pose.getTranslation();
        Translation2d tag = Field.APRIL_TAGS[id-1].toPose2d().getTranslation();
        
        return robot.getDistance(tag);
    }

    @Override
    public void periodic(){
        for (Result result : results) {
            SmartDashboard.putNumber("Vision/" + result.getAuthor() + "/X", result.getPose().getX());
            SmartDashboard.putNumber("Vision/" + result.getAuthor() + "/Y", result.getPose().getY());
            SmartDashboard.putNumber("Vision/" + result.getAuthor() + "/Degrees", result.getPose().getRotation().getDegrees());
        }
    }
}