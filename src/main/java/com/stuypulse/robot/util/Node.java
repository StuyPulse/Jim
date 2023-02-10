package com.stuypulse.robot.util;

public class Node {
    private double g;
    private double h;
    private double f;
    private int shoulderAngle; // row
    private int wristAngle; // col
    private boolean isBarrier;
    private Node parent;

    public Node(int shoulderAngle, int wristAngle) {
        this.shoulderAngle = shoulderAngle;
        this.wristAngle = wristAngle;
    }
    
    /*
    * Getter Functions
    */

    public double getH() {
        return this.h;
    }

    public double getF() {
        return this.f;
    }

    public double getG() {
        return this.g;
    }

    public Node getParent() {
        return this.parent;
    }

    public int getShoulderAngle() {
        return this.shoulderAngle;
    }

    public int getWristAngle() {
        return this.wristAngle;
    }

    public boolean isBarrier() {
        return isBarrier;
    }

    public String toString() {
        return "Node [row=" + shoulderAngle + ", col=" + wristAngle + "]";
    }

    /*
    * Setter Functions
    */
    public void setH(double h) {
        this.h = h;
    }

    public void setG(double g) {
        this.g = g;
    }

    public void setF(double f) {
        this.f = f;
    }

    public void setParent(Node parent) {
        this.parent = parent;
    }

    public void setBarrier(Boolean isbarrier) {
        this.isBarrier = isbarrier;
    }

    public void setShoulderAngle(int row) {
        this.shoulderAngle = row;
    }

    public void setWristAngle(int column) {
        this.wristAngle = column;
    }

    @Override
    public boolean equals(Object o) {
        Node node = (Node) o;
        return this.shoulderAngle == node.getShoulderAngle() && this.wristAngle == node.getWristAngle();
    }

    public void calculateFinalCost() {
        setF(getG() + getH()); 
    }

    public void updateNode(Node currNode, double cost) {
        double ccost = currNode.getG() + cost;
        setParent(currNode);
        setG(ccost);
        calculateFinalCost();
    }

    public boolean checkBetterPath(Node currNode, double cost) {
        double ccost = currNode.getG() + cost;
        if (ccost < getG()) {
            updateNode(currNode, cost);
            return true;
        }
        return false;
    }
    // calculate the cost 
    public void calculateH(Node endNode) {
        this.h = Math.pow(Math.pow(endNode.getShoulderAngle() - getShoulderAngle(), 2) + Math.pow(endNode.getWristAngle(), -getWristAngle()), 0.5);
    }

}