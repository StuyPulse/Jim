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


    // TODO: Take in resolution multiplier
    public static ArmTrajectory generateTrajectory(int resolutionx, int resolutiony, double initialShoulderAngle, double initialWristAngle, double finalShoulderAngle, double finalWristAngle) {
        // IMPORTANT RES X RES Y should be 30 and 120
        Asstar astar = new Asstar(resolutionx, resolutiony, new Node((int) (initialShoulderAngle * Settings.Arm.AStar.RESOLUTION_MULTIPLIER), (int) (initialWristAngle * Settings.Arm.AStar.RESOLUTION_MULTIPLIER)), new Node((int) (finalShoulderAngle * Settings.Arm.AStar.RESOLUTION_MULTIPLIER), (int) (finalWristAngle * Settings.Arm.AStar.RESOLUTION_MULTIPLIER)));
        int[][] blocksArray = new int[][]{{60, 0}, {60, 1}, {60, 2}, {60, 3},{60, 4}, {60, 5}, {60, 6}, {60, 7}, {60, 8}, {60, 9},{60, 10}, 
                                          {60, 11}, {60, 12}, {60, 13},{60, 14}, {60, 15}, {60, 16}, {60, 17}, {60, 18}, {60, 19},{60, 20},
                                          {60, 21}, {60, 22}, {60, 23},{60, 24}, {60, 25}, {60, 26}, {60, 27}, {60, 28}, {60, 29},{60, 30},
                                          {60, 31}, {60, 32}, {60, 33},{60, 34}, {60, 35}, {60, 36}, {60, 37}, {60, 38}, {60, 39},{60, 40},
                                          {60, 41}, {60, 42}, {60, 43},{60, 44}, {60, 45}, {60, 46}, {60, 47}, {60, 48}, {60, 49},{60, 50},
                                          {61, 50}, {62, 50}, {63, 50}, {64, 50}, {65, 50}, {66, 50}, {67, 50}, {68, 50}, {69, 50}
                                        };
        astar.setBlocks(blocksArray);
        List<Node> path = new ArrayList<Node>();
        for (Node node : astar.findPath()) {
            System.out.println(node);
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
