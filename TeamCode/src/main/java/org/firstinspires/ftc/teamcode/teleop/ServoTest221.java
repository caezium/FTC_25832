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
    // Servo declarations
    ServoImplEx armLeft, armRight, clawServo, frontServo, turnHeadServo;

    // PWM ranges for different servos (500-2500 is full range)
    PwmControl.PwmRange armRange = new PwmControl.PwmRange(500, 1250);    // 0: down, 1: up
    PwmControl.PwmRange clawRange = new PwmControl.PwmRange(500, 1150);   // 0: open, 1: close
    PwmControl.PwmRange frontRange = new PwmControl.PwmRange(500, 2500);  // 0: down, 1: up
    PwmControl.PwmRange turnHeadRange = new PwmControl.PwmRange(500, 2500); // Full range for head rotation

    @Override
    public void runOpMode() {
        // Initialize servos
        armLeft = hardwareMap.get(ServoImplEx.class, "long1");
        armRight = hardwareMap.get(ServoImplEx.class, "long2");
        clawServo = hardwareMap.get(ServoImplEx.class, "claw");
        frontServo = hardwareMap.get(ServoImplEx.class, "front");
        turnHeadServo = hardwareMap.get(ServoImplEx.class, "turnhead");

        // Set servo directions
        armLeft.setDirection(Servo.Direction.FORWARD);
        armRight.setDirection(Servo.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.FORWARD);
        frontServo.setDirection(Servo.Direction.FORWARD);
        turnHeadServo.setDirection(Servo.Direction.FORWARD);

        // Set PWM ranges for each servo
        armLeft.setPwmRange(armRange);
        armRight.setPwmRange(armRange);
        clawServo.setPwmRange(clawRange);
        turnHeadServo.setPwmRange(turnHeadRange);
        frontServo.setPwmRange(frontRange);

        waitForStart();

        while (opModeIsActive()) {
            // Control logic (currently commented out)
//            armLeft.setPosition(gamepad1.left_stick_y);
//            armRight.setPosition(gamepad1.left_stick_y);
//            turnHeadServo.setPosition(gamepad1.right_stick_x);
//            frontServo.setPosition(gamepad1.right_stick_y);
//            clawServo.setPosition(gamepad1.left_trigger);
        }
    }
}