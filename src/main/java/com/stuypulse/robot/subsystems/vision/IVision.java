package com.stuypulse.robot.subsystems.vision;

import java.util.List;

import com.stuypulse.robot.util.AprilTagData;
import com.stuypulse.robot.util.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IVision extends SubsystemBase {

    /** SINGLETON **/
    private static IVision instance;

    public static IVision getInstance() {
        if(instance == null) {
            instance = new Vision();
        } 
        return instance; 
    }

    /** VISION TYPES **/
    public enum Noise {
        LOW,
        MID,
        HIGH;
    }

    public static class Result {
        public final AprilTagData data;
        public final Noise noise;
        public final Limelight author;

        public Result(AprilTagData data, Noise error, Limelight author) {
            this.data = data;
            this.noise = error;
            this.author = author;
        }

        public Pose2d getPose() {
            return data.pose;
        }

        public double getLatency() {
            return data.latency;
        }

        public Noise getNoise() {
            return noise;
        }

        public Limelight getLimelight(){
            return author;
        }
    }

    /** ABSTRACT METHODS **/
    public abstract List<Result> getResults();
}
