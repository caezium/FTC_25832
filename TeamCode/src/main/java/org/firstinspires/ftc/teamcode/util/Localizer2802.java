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


public class Localizer2802{
    public static DcMotor podL, podR, podM; // pod left, pod right, pod middle.
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

    public void positionArc(){
        double dX, dY;
        leftEncoderPos = -podL.getCurrentPosition(); // negative
        rightEncoderPos = -podR.getCurrentPosition();
        middleEncoderPos = podM.getCurrentPosition();

        deltaLeftEncoderPos = leftEncoderPos - previousLeftEncoderPos;
        deltaRightEncoderPos = rightEncoderPos - previousRightEncoderPos;
        deltaMiddleEncoderPos = middleEncoderPos - previousMiddleEncoderPos;

        deltaLeftEncoderPos *= C;
        deltaRightEncoderPos *= C;
        deltaMiddleEncoderPos *= C;

        // double phi = (deltaLeftEncoderPos-deltaRightEncoderPos) / (Ly-Ry);
        // double deltaMiddlePos = (deltaLeftEncoderPos + deltaRightEncoderPos) / 2;
        // double deltaPerpPos = deltaMiddleEncoderPos - Bx * phi;


        // xPos += dX;
        // yPos += dY;
        // heading += phi;

        previousMiddleEncoderPos = middleEncoderPos;
        previousLeftEncoderPos = leftEncoderPos;
        previousRightEncoderPos = rightEncoderPos;

        double fwd = (deltaRightEncoderPos*Ly-deltaLeftEncoderPos*Ry)/(Ly-Ry); //change in relative x direction
        double dA = (deltaRightEncoderPos-deltaLeftEncoderPos)/(Ly-Ry); //theta, the heading, delta angle
        double str = deltaMiddleEncoderPos - Bx*dA; //strafe
        //bx is the x position of the middle odometer
        //ly is the y position of the left odometer...

        //rX = relative x
//        double rX = fwd;
//        double rY = str;
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
}
