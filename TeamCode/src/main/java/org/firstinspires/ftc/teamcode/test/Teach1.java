package org.firstinspires.ftc.teamcode.test;

import java.util.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(group="Test")
public class Teach1 extends LinearOpMode {


    double up=2500;
    double cur=0;
    boolean flag=true;
    double lsta=System.currentTimeMillis();

    @Override
    public void runOpMode(){
        PwmControl.PwmRange range = new PwmControl.PwmRange(500, up);
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "swervo");
        servo.setPwmRange(range);

        waitForStart();

        while (opModeIsActive()) {
//            if (gamepad1.x && System.currentTimeMillis()-lsta >= 100) {
//                servo.setPosition(1-cur);
//                cur = 1-cur;
//                lsta = System.currentTimeMillis();
//            }
//                if (gamepad1.x) servo.setPosition(1);
//                else servo.setPosition(0);
               if (flag && System.currentTimeMillis() - lsta >= 500) {
                   cur += 0.05;
                   servo.setPosition(cur);
                   telemetry.addData("swervo", "(%.2f)", cur);
               }
               if (gamepad1.x) {
                   flag = false;
                   servo.setPosition(0);
               }
               if (cur >= 0.9) flag=false; 
        }
    }
}
