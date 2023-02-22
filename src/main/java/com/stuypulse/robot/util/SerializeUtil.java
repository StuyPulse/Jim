package com.stuypulse.robot.util;

public class SerializeUtil {
    public static byte[] boolToBytearr(boolean b) {
        return new byte[] { (byte) (b ? 1 : 0) };
    }
    
    public static boolean bytearrToBool(byte[] b) {
        return b[0] == 1;
    }

    public static byte[] doubleToBytearr(double d) {
        long l = Double.doubleToLongBits(d);

        byte[] result = new byte[8];
        for (int i = 7; i >= 0; i--) {
            result[i] = (byte)(l & 0xFF);
            l >>= 8;
        }
        
        return result;
    }

    public static double bytearrToDouble(byte[] b) {
        long l = 0;

        for (int i = 0; i < 8; i++) {
            l |= (b[i] & 0xff) << (8 * (7 - i));
        }

        return Double.longBitsToDouble(l);
    }
}
