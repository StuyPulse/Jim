package com.stuypulse.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Stack;

import javax.xml.transform.Source;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Pose2dGraph {
    private Map<Pose2d, List<Pose2d> > nodes;
    
    public Pose2dGraph() {
        this.nodes = new HashMap<>();
        numNodes = 0;
    }
    public Pose2dGraph(Map<Pose2d, List<Pose2d> > nodes) {
        this.nodes = nodes;
    }    

    public void addNode(Pose2d node) {
        nodes.put(node, new ArrayList<>());
    }

    public void addEdge(Pose2d src, Pose2d dst, boolean bidirectional) {
        if (!nodes.containsKey(src))
            addNode(src);
 
        if (!nodes.containsKey(dst))
            addNode(dst);

        nodes.get(src).add(dst);
        if (bidirectional) {
            nodes.get(dst).add(src);
        }
    }

    public boolean hasEdge(Pose2d src, Pose2d dst) {
        return nodes.get(src).contains(dst);
    }

    public Pose2d[] shortestPath(Pose2d source, Pose2d target) {
        Map<Pose2d, Double> sourceDistance = new HashMap<>(); 
        Map<Pose2d, Pose2d> prevNodes = new HashMap<>();
        Queue<Pose2d> nodeQueue = new LinkedList<>();

        for(Pose2d node : nodes.keySet()) {
            sourceDistance.put(node, Double.MAX_VALUE);
            prevNodes.put(node, null);
            nodeQueue.add(node);
        }
        sourceDistance.replace(source, 0.0);

        Pose2d current;
        while(!nodeQueue.isEmpty()) {
            current = nodeQueue.poll();
            if(current.equals(target)) break;

            for(Pose2d neighbor : nodes.get(current)) {
                double alt = sourceDistance.get(current) + current.getTranslation().getDistance(neighbor.getTranslation());
                if(alt < sourceDistance.get(neighbor)) {
                    sourceDistance.replace(neighbor, alt);
                    prevNodes.replace(neighbor, current);
                }
            }
        }

        Stack<Pose2d> path = new Stack<>();
        current = target;
        if(prevNodes.get(current) != null || current.equals(source)) {
            while(current != null) {
                path.push(current);
                current = prevNodes.get(current);
            }
        }       

        Pose2d[] pathArray = new Pose2d[path.size()];
        for(int i = 0; i < pathArray.length; i++) {
            pathArray[i] = path.pop();
        }
        return pathArray;

    }
    
        

    
}