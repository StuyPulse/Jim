package com.stuypulse.robot.util;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;


public class Astar {

    public static final int DEGREE_RANGE = 360;

    public static int normDegrees(int deg) {
        return deg - DEGREE_RANGE * (int)Math.round((double)(deg - (DEGREE_RANGE / 2)) / DEGREE_RANGE);
    }
    
    public static int normDegreesDistance(int deg) {
        return deg - DEGREE_RANGE * (int)Math.round((double)deg / DEGREE_RANGE);
    }


    private int hCost;
    private int diagCost;
    private Node[][] searchArea;
    private Queue<Node> openList;
    private Set<Node> closedSet;
    private Node initialNode;
    private Node finalNode;
    private Constraint constraint; 

    public static void main(String[] args) {
        for (int i = -400; i < 400; i+=5) {
            System.out.println(normDegrees(i) + " hhe" + i);
        }
    }

    public Astar(Node initialNode, Node finalNode, int hvCost, int diagonalCost) {
        this.hCost = hvCost;
        this.diagCost = diagonalCost;
        setInitialNode(initialNode);
        setFinalNode(finalNode);
        this.searchArea = new Node[DEGREE_RANGE][DEGREE_RANGE];
        this.openList = new LinkedList<Node>();
        setNodes();
        this.closedSet = new HashSet<>();
        constraint = (s, w) -> false;
    }

    public Astar(Node initialNode, Node finalNode) {
        this(initialNode, finalNode, 10, 14);
    }

    public Astar addConstraint(Constraint newConstraint) {
        constraint = constraint.add(newConstraint);
        return this;
    }

    public Node getNode(int shoulderDeg, int wristDeg) {
        return searchArea[normDegrees(shoulderDeg)][normDegrees(wristDeg)];
    }

    public void setNode(int shoulderDeg, int wristDeg, Node node) {
        searchArea[normDegrees(shoulderDeg)][normDegrees(wristDeg)] = node;
    }

    private void setNodes() {
        for (int i = 0; i < DEGREE_RANGE; i++) {
            for (int j = 0; j < DEGREE_RANGE; j++) {
                Node node = new Node(i, j);
                node.calculateH((getFinalNode()));
                setNode(i, j, node);
            }
        }
    }

    public List<Node> findPath() {
        openList.add(initialNode);
        while (!isEmpty(openList)) {
            Node currentNode = openList.poll();
            closedSet.add(currentNode);
            if (isFinalNode(currentNode)) {
                return getPath(currentNode);
            } else {
                addAdjacentNodes(currentNode);
            }
        }
        return new ArrayList<Node>();
    }

    private List<Node> getPath(Node currentNode) {
        List<Node> path = new ArrayList<Node>();
        path.add(currentNode);
        Node parent;
        while ((parent = currentNode.getParent()) != null) {
            path.add(0, parent);
            currentNode = parent;
        }
        return path;
    }

    private void addAdjacentNodes(Node currentNode) {
        addAdjacentUpperRow(currentNode);
        addAdjacentMiddleRow(currentNode);
        addAdjacentLowerRow(currentNode);
    }

    private void addAdjacentLowerRow(Node currentNode) {
        int shoulder = currentNode.getShoulderAngle();
        int wrist = currentNode.getWristAngle();
        int lowerRow = shoulder + 1;
        checkNode(currentNode, wrist - 1, lowerRow, getDiagonalCost()); // Comment if diagonal not allowed
        checkNode(currentNode, wrist, lowerRow, getHCost());
        checkNode(currentNode, wrist + 1, lowerRow, getDiagonalCost()); // Comment if diagonal not allowed
    }

    private void addAdjacentMiddleRow(Node currentNode) {
        int shoulder = currentNode.getShoulderAngle();
        int wrist = currentNode.getWristAngle();
        int middleRow = shoulder;
        checkNode(currentNode, wrist - 1, middleRow, getHCost());
        checkNode(currentNode, wrist + 1, middleRow, getHCost());
    }

    private void addAdjacentUpperRow(Node currentNode) {
        int shoulder = currentNode.getShoulderAngle();
        int wrist = currentNode.getWristAngle();
        int upperRow = shoulder - 1;
        checkNode(currentNode, wrist - 1, upperRow, getDiagonalCost()); // Comment this if diagonal movements are not allowed
        checkNode(currentNode, wrist + 1, upperRow, getDiagonalCost()); // Comment this if diagonal movements are not allowed
        checkNode(currentNode, wrist, upperRow, getHCost());

    }

    private void checkNode(Node currentNode, int wrist, int shoulder, int cost) {
        Node adjacentNode = getNode(shoulder, wrist);
        if (!constraint.isInvalid(normDegreesDistance(shoulder), normDegreesDistance(wrist)) && !getClosedSet().contains(adjacentNode)) {
            if (!getOpenList().contains(adjacentNode)) {
                adjacentNode.updateNode(currentNode, cost);
                getOpenList().add(adjacentNode);
            } else {
                boolean changed = adjacentNode.checkBetterPath(currentNode, cost);
                if (changed) {
                    getOpenList().remove(adjacentNode);
                    getOpenList().add(adjacentNode);
                }
            }
        }
    }

    private boolean isFinalNode(Node currentNode) {
        return currentNode.equals(finalNode);
    }

    private boolean isEmpty(Queue<Node> openList) {
        return openList.size() == 0;
    }

    public Node getInitialNode() {
        return initialNode;
    }

    public void setInitialNode(Node initialNode) {
        this.initialNode = initialNode;
    }

    public Node getFinalNode() {
        return finalNode;
    }

    public void setFinalNode(Node finalNode) {
        this.finalNode = finalNode;
    }

    public Queue<Node> getOpenList() {
        return openList;
    }

    public void setOpenList(PriorityQueue<Node> openList) {
        this.openList = openList;
    }

    public Set<Node> getClosedSet() {
        return closedSet;
    }

    public void setClosedSet(Set<Node> closedSet) {
        this.closedSet = closedSet;
    }

    public int getHCost() {
        return hCost;
    }

    public void setHCost(int hvCost) {
        this.hCost = hvCost;
    }

    private int getDiagonalCost() {
        return diagCost;
    }
}
