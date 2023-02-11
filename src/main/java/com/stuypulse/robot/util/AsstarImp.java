package com.stuypulse.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.stuypulse.robot.constants.Settings;

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
        Asstar astar = new Asstar(resolutionx, resolutiony, new Node((int) (initialShoulderAngle * Settings.Arm.AStar.RESOLUTION_MULTIPLIER), (int) (initialWristAngle * Settings.Arm.AStar.RESOLUTION_MULTIPLIER)), new Node((int) (finalShoulderAngle * Settings.Arm.AStar.RESOLUTION_MULTIPLIER), (int) (finalWristAngle * Settings.Arm.AStar.RESOLUTION_MULTIPLIER)));
        List<Node> path = new ArrayList<Node>();
        for (Node node : astar.findPath()) {
            path.add(node);
        }
        List<ArmState> armstates = new ArrayList<>(path.size());
        for (Node node : path) {
            // DIVIDE BY RES MULTIPLIER TDLR
            armstates.add(new ArmState(new Rotation2d(Math.toRadians(-node.getShoulderAngle())), new Rotation2d(Math.toRadians(node.getShoulderAngle())))); // CHANGE IMPORTANT
        }
        return new ArmTrajectory().addStates(armstates);
    }
    



}
