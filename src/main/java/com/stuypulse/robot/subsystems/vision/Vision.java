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

    public Vision() {
        String[] hostNames = LIMELIGHTS;
        limelights = new Limelight[hostNames.length];

        for(int i = 0; i < hostNames.length; i++){
            limelights[i] = new Limelight(hostNames[i]);
            for (int port : PORTS) {
                PortForwarder.add(port, hostNames[i] + ".local", port);
            }
        }
    }

    public List<Result> getResults() {
        var results = new ArrayList<Result>();

        for (Limelight ll : limelights){
            var data = ll.getPoseData();

            if(data.isPresent()) {
                results.add(process(data.get()));
            }
        }
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


    private double distanceToTarget(Pose2d pose, int id) {
        Translation2d robot = pose.getTranslation();
        Translation2d tag = Field.APRIL_TAGS[id-1].toPose2d().getTranslation();
        
        return robot.getDistance(tag);
    }

    @Override
    public void periodic(){
        for (Limelight ll : limelights){
            String name = ll.getTableName();
            // TODO: check if camera is connected

            // SmartDashboard.putNumber("Vision/" + name +  "/Pose X", pose.getX());
            // SmartDashboard.putNumber("Vision/" + name +  "/Pose Y", pose.getY());
            // SmartDashboard.putNumber("Vision/" + name + "/Pose Rotation", pose.getRotation().getDegrees());   
        }
    }
}