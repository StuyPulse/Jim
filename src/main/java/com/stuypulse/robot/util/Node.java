package com.stuypulse.robot.util;

import javax.crypto.spec.GCMParameterSpec;

public class Node {
    private double g;
    private double h;
    private double f;
    private int row;
    private int column;
    private boolean isBarrier;
    private Node parent;

    public Node(int row, int column) {
        this.row = row;
        this.column = column;
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

    public int getRow() {
        return this.row;
    }

    public int getColumn() {
        return this.column;
    }

    public boolean isBarrier() {
        return isBarrier;
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

    public void setRow(int row) {
        this.row = row;
    }

    public void setColumn(int column) {
        this.column = column;
    }

    @Override
    public boolean equals(Object o) {
        Node node = (Node) o;
        return this.row == node.getRow() && this.column == node.getColumn();
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
        this.h = Math.pow(Math.pow(endNode.getRow() - getRow(), 2) + Math.pow(endNode.getColumn(), -getColumn()), 0.5);
    }

}