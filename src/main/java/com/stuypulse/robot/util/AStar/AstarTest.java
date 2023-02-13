package com.stuypulse.robot.util.AStar;

import java.util.List;


// TODO: REMOVE
public class AstarTest {

    public static void main(String[] args) {
        Node initialNode = new Node(2, 1);
        Node finalNode = new Node(350, 45);
        Astar aStar = new Astar(initialNode, finalNode);
        aStar.addConstraint((s, w) -> (s == 60) && (0 <= w && w <= 50));
        List<Node> path = aStar.findPath();
        System.out.println(path.size());

        for (Node node : path) {
            System.out.println(node);
        }

        
    }
}