package com.stuypulse.robot.util;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

public class FieldFileUtil {
    public static final String fileHeader = new StringBuilder()
        .append("package com.stuypulse.robot.constants.fields;\n\n")
        .append("public class %sBFS {\n")
        .append("\tpublic static int[] array = new int[] {\n")
        .toString();

    public static File getFieldDirectory() {
        return new File(
            Filesystem.getOperatingDirectory()
                + File.separator + "src"
                + File.separator + "main"
                + File.separator + "java"
                + File.separator + "com"
                + File.separator + "stuypulse"
                + File.separator + "robot"
                + File.separator + "constants"
                + File.separator + "fields");
    }

    public static File getFieldFile(String name) {
        return new File(
            Filesystem.getOperatingDirectory()
                + File.separator + "src"
                + File.separator + "main"
                + File.separator + "java"
                + File.separator + "com"
                + File.separator + "stuypulse"
                + File.separator + "robot"
                + File.separator + "constants"
                + File.separator + "fields"
                + File.separator + name + "BFS.java");
    }
}
