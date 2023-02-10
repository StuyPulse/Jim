package com.stuypulse.robot.util;

import java.util.List;

public class AsstarImp {

    private int resolutionx;
    private int resolutiony;
    private Node initialPosition;
    private Node finalPosition;

    private Asstar asstar;

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
    


}
