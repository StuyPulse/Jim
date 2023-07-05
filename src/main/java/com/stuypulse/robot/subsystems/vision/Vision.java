/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.vision;


import com.stuypulse.robot.util.AprilTagData;
import com.stuypulse.robot.util.Limelight.DataType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

public abstract class Vision extends SubsystemBase {

    /** SINGLETON **/
    private static final Vision instance;

    static {
        instance = new VisionImpl();
    }

    public static Vision getInstance() {
        return instance;
    }

    protected Vision() {
    }

    /** VISION TYPES **/

    /** ABSTRACT METHODS **/
    public abstract List<AprilTagData> getResults();

    public abstract double getDistanceToPeg();
    public abstract double getDistanceToCube();
    public abstract double getAngle();

    public abstract DataType getPipeline();
    public abstract void setPipeline(DataType type);
}
