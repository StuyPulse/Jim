package com.stuypulse.robot.util;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

public class FieldFileUtil {
    public static File getFieldDirectory() {
        return new File(
            Filesystem.getLaunchDirectory(), 
            "src"
                + File.separator
                + "main"
                + File.separator
                + "deploy"
                + File.separator
                + "fields");
    }

    public static File getFieldFile(String name) {
        return new File(
            Filesystem.getLaunchDirectory(),
            "src"
                + File.separator
                + "main"
                + File.separator
                + "deploy"
                + File.separator
                + "fields"
                + File.separator
                + name + ".BFS");
    }
}
