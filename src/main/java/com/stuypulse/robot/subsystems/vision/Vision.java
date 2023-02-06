package com.stuypulse.robot.subsystems.vision;
import java.util.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.util.AprilTagData;
import com.stuypulse.robot.util.Limelight;
import com.stuypulse.stuylib.network.SmartNumber;

import static com.stuypulse.robot.constants.Settings.Vision.Limelight.*;
import static com.stuypulse.robot.constants.Settings.Vision.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends IVision {

    private final Limelight[] limelights;

    private final Field2d cameraField2d;

    public Vision() {
        String[] hostNames = LIMELIGHTS;
        limelights = new Limelight[hostNames.length];

        for(int i = 0; i < hostNames.length; i++){
            limelights[i] = new Limelight(hostNames[i]);
            for (int port : PORTS) {
                PortForwarder.add(port, hostNames[i] + ".local", port);
            }
        }

        cameraField2d = new Field2d();
        SmartDashboard.putData(cameraField2d);
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
        double angleDegrees = absDegToTarget(data.pose, data.id);
        double distance = distanceToTarget(data.pose, data.id);

        SmartDashboard.putNumber("Vision/Angle to Tag", angleDegrees);
        SmartDashboard.putNumber("Vision/Distance", distance);
        SmartDashboard.putNumber("Vision/Tag ID", data.id);
        
        // SmartDashboard.putNumber("Vision/Pose Degrees", data.pose.getRotation().getDegrees());
        // SmartDashboard.putNumber("Vision/Pose X", data.pose.getX());
        // SmartDashboard.putNumber("Vision/Pose Y", data.pose.getY());


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
        for (Limelight ll : limelights) {
            // if (!ll.isConnected()) {
            //     System.out.println("[Error] Limelight " + ll.getTableName() + " is not connected!");
            //     continue;
            // }

            String name = ll.getTableName();

            Optional<AprilTagData> data = ll.getPoseData();

            SmartDashboard.putBoolean("Vision/Is Connected", data.isPresent());

            if (data.isPresent()) {
                Pose2d pose = data.get().pose;

                this.process(data.get());
                SmartDashboard.putNumber("Vision/" + name +  "/Pose X", pose.getX());
                SmartDashboard.putNumber("Vision/" + name +  "/Pose Y", pose.getY());
                SmartDashboard.putNumber("Vision/" + name + "/Pose Rotation", pose.getRotation().getDegrees());
                

            } else {
                SmartDashboard.putNumber("Vision/" + name +  "/Pose X", Double.NaN);
                SmartDashboard.putNumber("Vision/" + name +  "/Pose Y", Double.NaN);
                SmartDashboard.putNumber("Vision/" + name + "/Pose Rotation", Double.NaN);   
            }
        }
    }
} 