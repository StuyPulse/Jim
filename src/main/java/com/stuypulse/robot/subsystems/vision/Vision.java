package com.stuypulse.robot.subsystems.vision;

import java.util.List;

import com.stuypulse.robot.util.AprilTagData;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Vision extends SubsystemBase {

    /** SINGLETON **/
    private static Vision instance;

    public static Vision getInstance() {
        if(instance == null) {
            instance = new VisionImpl();
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
        public Noise noise;

        public Result(AprilTagData data, Noise error) {
            this.data = data;
            this.noise = error;
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

        public AprilTagData getData(){
            return data;
        }
    }

    /** ABSTRACT METHODS **/
    public abstract List<Result> getResults();
}
