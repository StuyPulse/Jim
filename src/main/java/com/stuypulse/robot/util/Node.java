package com.stuypulse.robot.util;

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

    // public boolean checkBetterPath(Node currNode, double cost) {
    //     double ccost = currNode.getH();
    // }
    // calculate the cost 
    public void calculateH(Node endNode) {
        
    }

}