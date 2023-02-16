package com.stuypulse.robot.util.AStar;

import java.awt.*;
import java.awt.geom.Area;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;

public class AngleCalculator {

    double TOWER_HEIGHT = 48;
    double SHOULDER_WIDTH = 1;
    double SHOULDER_LENGTH = 42;

    double WRIST_WIDTH = 1;
    double WRIST_LENGTH = 16;
    
    double shoulder_x = 0;
    double shouder_y = TOWER_HEIGHT;
    
    double wrist_x = 0;
    double wrist_y = 0;

    double distance = 0;
    Polygon bumper = new Polygon();

    ArrayList<Ting> badSAngles = new ArrayList<>();

    

    public Polygon getPolygonFromMidline(double midline_x, double midline_y, double angle, double width, double length) {
        angle *= (Math.PI) / 180;

        double P1x = (width/2) * Math.cos((Math.PI / 2) + angle) + midline_x;
        double P1y = (width/2) * Math.sin((Math.PI / 2) + angle) + midline_y;

        double P2x = midline_x - (width/2) * Math.cos((Math.PI / 2) + angle);
        double P2y = midline_y - (width/2) * Math.sin((Math.PI / 2) + angle);

        double MIDX2 = length * Math.cos(angle) + P1x;
        double MIDY2 = length * Math.sin(angle) + P2y;


        this.wrist_x = MIDX2;
        this.wrist_y = MIDY2;

        double P3x = P2x + Math.cos(-angle) * length;
        double P3y = P2y - Math.sin(-angle) * length;


        double P4x = P1x + Math.cos(-angle) * length;
        double P4y = P1y - Math.sin(-angle) * length;

        Polygon p = new Polygon();

        p.addPoint((int) P1x, (int) P1y);
        p.addPoint((int) P2x, (int) P2y);
        p.addPoint((int) P3x, (int) P3y);
        p.addPoint((int) P4x, (int) P4y);
        System.out.println(P1x + " " + P1y + " | " + P2x + " " + P2y + " | " + P3x + " " + P3y + " | " + P4x + " " + P4y);
        return p;

    }


    public void init() {
        bumper.addPoint(-12, 0);
        bumper.addPoint(12, 0);
        bumper.addPoint(-12, 4);
        bumper.addPoint(12, 4);
    }


    public static void main(String[] args) {
        AngleCalculator a = new AngleCalculator();
        a.init();
        for (int i = -60; i < -59; i++) {
            for (int j = -180; j < 0; j++) {
                Polygon p1 = a.getPolygonFromMidline(a.shoulder_x, a.shouder_y, i, a.SHOULDER_WIDTH, a.SHOULDER_LENGTH);
                Polygon p2 = a.getPolygonFromMidline(a.wrist_x, a.wrist_y, j, a.WRIST_WIDTH, a.WRIST_LENGTH);
                Area o = new Area(p2);
                o.intersect(new Area(a.bumper));
                Area z = new Area(p1);
                z.intersect(new Area(a.bumper));
                if (!o.isEmpty() || !z.isEmpty()) {
                    a.badSAngles.add(new Ting(i, j));
                    System.out.println(true);
                }
            }
        }
        try {
            FileWriter myWriter = new FileWriter("filename.txt");
            for (Ting ting : a.badSAngles) {
                myWriter.write(ting.toString() + "\n");

            }
            myWriter.close();
            System.out.println("Successfully wrote to the file.");
          } catch (Exception e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
          }
        



    }
}
