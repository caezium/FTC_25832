package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(group="Test")
public class TestLower extends LinearOpMode {
    ServoImplEx part1, part2, part3, slide1, slide2;

    PwmControl.PwmRange v4range = new PwmControl.PwmRange(500, 2500);
    PwmControl.PwmRange downrange = new PwmControl.PwmRange(500, 900);

    @Override
    public void runOpMode() {
        part1 = hardwareMap.get(ServoImplEx.class, "apple");
        part2 = hardwareMap.get(ServoImplEx.class, "bob");
        part3 = hardwareMap.get(ServoImplEx.class, "cyliis");

        slide1 = hardwareMap.get(ServoImplEx.class, "down1");
        slide2 = hardwareMap.get(ServoImplEx.class, "down2");
        part2.setDirection(ServoImplEx.Direction.REVERSE);
        part1.setPwmRange(v4range);
        part2.setPwmRange(v4range);
        part3.setPwmRange(v4range);

        slide2.setDirection(ServoImplEx.Direction.REVERSE);
        slide1.setPwmRange(downrange);
        slide2.setPwmRange(downrange);

        waitForStart();

        while(opModeIsActive()){
            part1.setPosition(gamepad1.left_trigger);
            part2.setPosition(gamepad1.left_trigger);
            part3.setPosition(gamepad1.right_trigger);

            slide1.setPosition(gamepad1.left_stick_x);
            slide2.setPosition(gamepad1.left_stick_x);


            telemetry.addData("part1", part1.getPosition());
            telemetry.addData("part2",part2.getPosition());
            telemetry.addData("part3",part3.getPosition());

            telemetry.addData("slide1",slide1.getPosition());
            telemetry.addData("slide2",slide2.getPosition());


            telemetry.update();
        }
    }

}
