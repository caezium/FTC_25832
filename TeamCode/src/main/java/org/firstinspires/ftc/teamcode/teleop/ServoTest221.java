/*
Copyright 2024 FIRST Tech Challenge Team 25832

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(group="TeleOp")
public class ServoTest221 extends LinearOpMode {
    ServoImplEx long1, long2, claw, front, turnhead;
    PwmControl.PwmRange longrange = new PwmControl.PwmRange(500,  1250);
    PwmControl.PwmRange clawrange = new PwmControl.PwmRange(500, 1150);
    PwmControl.PwmRange fullrange = new PwmControl.PwmRange(500, 2500);
    @Override
    public void runOpMode() {
        long1 = hardwareMap.get(ServoImplEx.class, "long1");
        long2 = hardwareMap.get(ServoImplEx.class, "long2");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        front = hardwareMap.get(ServoImplEx.class, "front");
        turnhead = hardwareMap.get(ServoImplEx.class, "turnhead");
        long1.setDirection(Servo.Direction.FORWARD);
        long2.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        front.setDirection(Servo.Direction.FORWARD);
        turnhead.setDirection(Servo.Direction.FORWARD);
        long1.setPwmRange(longrange);
        long2.setPwmRange(longrange);
        claw.setPwmRange(clawrange);
        turnhead.setPwmRange(fullrange);
        front.setPwmRange(fullrange);
        waitForStart();

        while (opModeIsActive()) {
            long1.setPosition(gamepad1.left_stick_y);
            long2.setPosition(gamepad1.left_stick_y);
            turnhead.setPosition(gamepad1.right_stick_x);
            front.setPosition(gamepad1.right_stick_y);
            claw.setPosition(gamepad1.left_trigger);

        }
    }
}
