package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Point;
import java.util.ArrayList;
import java.util.List;


public class Localizer{
    public static DcMotor podL, podR, podM; //pod left, pod right, pod middle.
    HardwareMap hardwareMap;
    public static double C = (35 * 3.14)/8192;
    public static double Ly = 154.1;
    public static double Ry = -151.4;
    public static double Bx = 35;

    public static double X = 0;
    public static double Y = 0;
    public static double theta = 0;

    public static long previousLeftEncoderPos, previousRightEncoderPos, previousMiddleEncoderPos, leftEncoderPos, rightEncoderPos, middleEncoderPos;
    public static double deltaLeftEncoderPos, deltaRightEncoderPos, deltaMiddleEncoderPos;

    IMU imu;
    
    public void initialize(HardwareMap map){
        hardwareMap = map;
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        podR = hardwareMap.get(DcMotor.class, "podR");
        podL = hardwareMap.get(DcMotor.class, "podL");
        podM = hardwareMap.get(DcMotor.class, "podM");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }


    public void resetIMU(){ imu.resetYaw(); }

    public void resetEncoder(){
        X = 0;
        Y = 0;
        theta = 0;
        podL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        podR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        podM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        podL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        podR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        podM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double heading(){ return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); }

    public static void positionArc(){
        double dX, dY;
        L = -podL.getCurrentPosition(); //negative
        R = -podR.getCurrentPosition();
        M = podM.getCurrentPosition();

        deltaLeftEncoderPos = leftEncoderPos - previousLeftEncoderPos;
        deltaRightEncoderPos = rightEncoderPos - previousRightEncoderPos;
        deltaMiddleEncoderPos = middleEncoderPos - previousMiddleEncoderPos;
        
        deltaLeftEncoderPos *= C;
        deltaRightEncoderPos *= C;
        deltaMiddleEncoderPos *= C;

        phi = (deltaLeftEncoderPos-deltaRightEncoderPos) / trackwidth;
        deltaMiddlePos = (deltaLeftEncoderPos + deltaRightEncoderPos) / 2;
        deltaPerpPos = deltaMiddleEncoderPos - forward_offset * phi;

        dX = deltaMiddlePos * cos(heading) - deltaPerpPos * sin(heading);
        dY = deltaMiddlePos * sin(heading) + deltaPerpPos * cos(heading);

        xPos += dX;
        yPos += dY;
        heading += phi;

        previousMiddleEncoderPos = middleEncoderPos;
        previousLeftEncoderPos = leftEncoderPos;
        previousRightEncoderPos = rightEncoderPos;

        double fwd = (deltaRightEncoderPos*Ly-deltaLeftEncoderPos*Ry)/(Ly-Ry); //change in relative x direction
        double dA = (deltaRightEncoderPos-deltaLeftEncoderPos)/(Ly-Ry); //theta, the heading, delta angle
        double str = deltaMiddleEncoderPos - Bx*dA; //strafe
        //bx is the x position of the middle odometer
        //ly is the y position of the left odometer...

        //rX = relative x
        rX = fwd;
        rY = str;
        double r0 = fwd/dA;
        double r1 = str/dA;


        //relative x and y
        double relX = r0*Math.sin(dA) - r1*(1-Math.cos(dA));
        double relY = r1*Math.sin(dA) + r0*(1-Math.cos(dA));

        dX = relX * Math.cos(theta) - relY * Math.sin(theta);
        dY = relY * Math.cos(theta) + relX * Math.sin(theta);

        //upd x and upd y, ipdf theta

        X += dX;
        Y += dY;
        theta += dA;
    }

    public static double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public static ArrayList<Point> intersect(Point center, double r, Point line1, Point line2) {
        ArrayList<Point> all = new ArrayList<>();
        if(line2.x-line1.x == 0){
            double y1 = Math.sqrt(Math.pow(r,2)-Math.pow(line1.x - center.x, 2));
            double y2 = -Math.sqrt(Math.pow(r,2)-Math.pow(line1.x - center.x, 2));
            y1 += center.y;
            y2 += center.y;
            all.add(new Point(line1.x, y1));
            all.add(new Point(line1.x, y2));
            return all;
        }

        double m1 = (line2.y - line1.y)/(line2.x - line1.x);
        double quadA = 1 + Math.pow(m1,2);

        double x1 = line1.x - center.x;
        double y1 = line1.y - center.y;

        double quadB = (2 * m1 * y1) - (2 * Math.pow(m1,2) * x1);

        double quadC = (Math.pow(m1,2) * Math.pow(x1, 2) - (2 * y1 * m1 * x1) + Math.pow(y1,2) - Math.pow(r, 2));



        try{
            double xRoot1 = (-quadB + Math.sqrt(Math.pow(quadB, 2) - 4 * quadA *quadC))/(2 * quadA);
            double xRoot2 = (-quadB - Math.sqrt(Math.pow(quadB, 2) - 4 * quadA *quadC))/(2 * quadA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot1 += center.x;
            yRoot1 += center.y;
            xRoot2 += center.x;
            yRoot2 += center.y;

            double minX = Math.min(line1.x, line2.x);
            double maxX = Math.max(line1.x, line2.x);

            if(xRoot1 > minX && xRoot1 < maxX){ all.add(new Point(xRoot1, yRoot1)); }
            if(xRoot2 > minX && xRoot2 < maxX){ all.add(new Point(xRoot2, yRoot2)); }

        }catch(Exception e){

        }
        return all;
    }
}
