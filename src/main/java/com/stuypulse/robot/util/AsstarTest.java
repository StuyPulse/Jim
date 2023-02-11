package com.stuypulse.robot.util;

import java.util.List;


// TODO: REMOVE
public class AsstarTest {

    public static void main(String[] args) {
        Node initialNode = new Node(2, 1);
        Node finalNode = new Node(65, 45);
        int rows = 90;
        int cols = 360;
        Asstar aStar = new Asstar(rows, cols, initialNode, finalNode);
        int[][] blocksArray = new int[][]{{60, 1}, {60, 2}, {60, 3},{60, 4}, {60, 5}, {60, 6}, {60, 7}, {60, 8}, {60, 9},{60, 10}, 
                                          {60, 11}, {60, 12}, {60, 13},{60, 14}, {60, 15}, {60, 16}, {60, 17}, {60, 18}, {60, 19},{60, 20},
                                          {60, 21}, {60, 22}, {60, 23},{60, 24}, {60, 25}, {60, 26}, {60, 27}, {60, 28}, {60, 29},{60, 30},
                                          {60, 31}, {60, 32}, {60, 33},{60, 34}, {60, 35}, {60, 36}, {60, 37}, {60, 38}, {60, 39},{60, 40},
                                          {60, 41}, {60, 42}, {60, 43},{60, 44}, {60, 45}, {60, 46}, {60, 47}, {60, 48}, {60, 49},{60, 50}
                                        };
        aStar.setBlocks(blocksArray);
        List<Node> path = aStar.findPath();
        System.out.println(path.size());

        for (Node node : path) {
            System.out.println(node);
        }

        
    }
}