/************************ PROJECT OFSS ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.util.CustomCamera;
import com.stuypulse.robot.util.Fiducial;
import com.stuypulse.robot.util.VisionData;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

public class Vision extends AbstractVision {

    private final CustomCamera[] cameras;
    private final FieldObject2d[] cameraPoses2D;
    private final List<VisionData> outputs;

    protected Vision() {
        String[] cameraNames = new String[] {"default"};
        Pose3d[] cameraLocations = new Pose3d[] {
            new Pose3d(Units.inchesToMeters(-12.5), Units.inchesToMeters(11.5), Units.inchesToMeters(8.5), 
            // new Pose3d(Units.inchesToMeters(0), Units.inchesToMeters(-11.5), Units.inchesToMeters(0), 
            // new Pose3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0), 
            new Rotation3d(0, 0, 180))
        };

        cameras = new CustomCamera[cameraNames.length];
        cameraPoses2D = new FieldObject2d[cameraNames.length];

        Field2d field = AbstractOdometry.getInstance().getField();

        for (int i = 0; i < cameraNames.length; i++) {
            cameras[i] = new CustomCamera(cameraNames[i], cameraLocations[i]);
            cameraPoses2D[i] = field.getObject(cameraNames[i]);
        }

        outputs = new ArrayList<>();
    }

    @Override
    public List<VisionData> getOutput() {
        return outputs;
    }

    private static void putAprilTagData(String prefix, VisionData data) {
        SmartDashboard.putNumber(prefix + "/Pose X", data.robotPose.getX());
        SmartDashboard.putNumber(prefix + "/Pose Y", data.robotPose.getY());
        SmartDashboard.putNumber(prefix + "/Pose Z", data.robotPose.getZ());

        SmartDashboard.putNumber(prefix + "/Distance to Tag", data.calculateDistanceToTag(data.getPrimaryTag()));

        SmartDashboard.putNumber(
                prefix + "/Pose Rotation",
                Units.radiansToDegrees(data.robotPose.getRotation().getAngle()));
        SmartDashboard.putNumber(prefix + "/Latency", data.latency);
    }

    @Override
    public void periodic() {
        outputs.clear();

        for (int i = 0; i < cameras.length; ++i) {
            CustomCamera camera = cameras[i];
            FieldObject2d cameraPose2D = cameraPoses2D[i];

            camera.updateData();

            if (camera.hasData()) {
                VisionData data = camera.getVisionData();
                outputs.add(data);

                Fiducial tag = data.getPrimaryTag();
                if (tag != null)
                    AbstractOdometry.getInstance().getField().getObject("Fiducial").setPose(tag.getPose().toPose2d());

                putAprilTagData("Vision/" + camera.getCameraName(), data);
                cameraPose2D.setPose(data.robotPose.toPose2d());
            }
            SmartDashboard.putBoolean("Vision/Has Data", camera.hasData());
        }
    }
}
