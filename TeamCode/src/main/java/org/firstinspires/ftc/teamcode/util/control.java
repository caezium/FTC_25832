package org.firstinspires.ftc.teamcode.util;

public class control {
    private static String motorport[] = {
      "",
      "",
      "",
      ""
    };

    private static String servoport[] = {
            "",
            "",
            "",
            ""
    };
    public static String motor(int port){ return motorport[port]; }
    public static String servo(int port){ return servoport[port]; }

}
