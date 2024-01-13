/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.util.CustomCamera;
import com.stuypulse.robot.util.Fiducial;
import com.stuypulse.robot.util.VisionData;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

public class Vision extends AbstractVision {

    private final CustomCamera[] cameras;
    private List<VisionData> outputs;

    protected Vision() {

        cameras = new CustomCamera[Cameras.ROBOT_CAMERAS.length];


        for (int i = 0; i < Cameras.ROBOT_CAMERAS.length; i++) {
            cameras[i] = Cameras.ROBOT_CAMERAS[i].getCamera();
        }

        outputs = new ArrayList<>();
    }

    @Override
    public List<VisionData> getOutput() {
        return outputs;
    }

    @Override
    public void setCameraLayouts(int[] tids) {
        for (CustomCamera camera : cameras) {
            camera.setLayout(Field.getFiducialLayout(tids));
        }
    }

    @Override
    public void periodic() {

        outputs = new ArrayList<>();

        for (CustomCamera camera: cameras) {
            camera.getVisionData().ifPresent(data -> {
                outputs.add(data);

                Fiducial tag = data.getPrimaryTag();

                AbstractOdometry.getInstance().getField().getObject("Fiducial").setPose(tag.getPose().toPose2d());

                String prefix = "Vision/" + camera.getCameraName();

                SmartDashboard.putNumber(prefix + "/Pose X", data.robotPose.getX());
                SmartDashboard.putNumber(prefix + "/Pose Y", data.robotPose.getY());
                SmartDashboard.putNumber(prefix + "/Pose Z", data.robotPose.getZ());

                SmartDashboard.putNumber(prefix + "/Distance to Tag", data.calculateDistanceToTag(data.getPrimaryTag()));

                SmartDashboard.putNumber(
                        prefix + "/Pose Rotation",
                        Units.radiansToDegrees(data.robotPose.getRotation().getAngle()));
                SmartDashboard.putNumber(prefix + "/Timestamp", data.timestamp);
            });
        }

        SmartDashboard.putBoolean("Vision/Has Any Data", !outputs.isEmpty());
    }
}
