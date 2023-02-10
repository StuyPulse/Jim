package com.stuypulse.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;

public class AsstarImp {

    private int resolutionx;
    private int resolutiony;
    private Node initialPosition;
    private Node finalPosition;

    private Asstar asstar;
    private ArmTrajectory armTrajectory;

    public AsstarImp(int resolutionx, int resolutiony, double initialShoulderAngle, double initialWristAngle, double finalShoulderAngle, double finalWristAngle) {
        update(resolutionx, resolutiony, initialShoulderAngle, initialWristAngle, finalShoulderAngle, finalWristAngle);
    }

    public List<Node> getTrajectory() {
        return asstar.findPath();
    }

    public void update(int resolutionx, int resolutiony, double initialShoulderAngle, double initialWristAngle, double finalShoulderAngle, double finalWristAngle) {
        this.resolutionx = resolutionx;
        this.resolutiony = resolutiony;
        this.initialPosition = new Node((int) initialShoulderAngle, (int) initialWristAngle);
        this.finalPosition = new Node((int) finalShoulderAngle, (int) finalWristAngle);
        asstar = new Asstar(resolutionx, resolutiony, initialPosition, finalPosition);
    }

    public void addConstraints() {
        // todo finish
    }



    public static ArmTrajectory generateTrajectory(int resolutionx, int resolutiony, double initialShoulderAngle, double initialWristAngle, double finalShoulderAngle, double finalWristAngle) {
        // IMPORTANT RES X RES Y should be 30 and 120
        Asstar asstar = new Asstar(resolutionx, resolutiony, new Node((int) initialShoulderAngle, (int) initialWristAngle), new Node((int) finalShoulderAngle, (int) finalWristAngle));
        List<Node> path = asstar.findPath();
        List<ArmState> armstates = new ArrayList<ArmState>();
        for (Node node : path) {
            armstates.add(new ArmState(new Rotation2d(3 * node.getShoulderAngle()), new Rotation2d(3 * node.getWristAngle()))); // CHANGE IMPORTANT
        }
        return new ArmTrajectory().addStates(armstates);
    }
    



}
