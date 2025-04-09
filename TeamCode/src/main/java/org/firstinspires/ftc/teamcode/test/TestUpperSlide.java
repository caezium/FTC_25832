package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.UpperSlide;
import org.firstinspires.ftc.teamcode.util.expansion;
import org.firstinspires.ftc.teamcode.util.control;

@TeleOp(group="Test")
public class TestUpperSlide extends LinearOpMode {

    UpperSlide slide = new UpperSlide();
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean cPressed = false;
    private boolean dPressed = false;
    @Override
    public void runOpMode() {
        slide.initialize(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.a){ slide.pos0(); }
            if(gamepad1.x){ slide.pos1(); }
            if(gamepad1.y){ slide.pos2(); }
            if(gamepad1.b){ slide.pos3(); }
            if(gamepad1.right_bumper) {
                if(!aPressed) {
                    slide.addArmPos(0.05);
                    aPressed = true;
                }
            } else {
                aPressed = false;
            }

            if(gamepad1.left_bumper) {
                if(!bPressed) {
                    slide.addArmPos(-0.05);
                    bPressed = true;
                }
            } else {
                bPressed = false;
            }

            if(gamepad1.right_trigger > 0) {
                if(!cPressed) {
                    slide.addSwingPos(0.05);
                    cPressed = true;
                }
            } else {
                cPressed = false;
            }

            if(gamepad1.left_trigger > 0) {
                if(!dPressed) {
                    slide.addSwingPos(-0.05);
                    dPressed = true;
                }
            } else {
                dPressed = false;
            }

            slide.updatePID();
            /*
            if(gamepad1.dpad_left){ slide.closeClaw(); }
            if(gamepad1.dpad_right){ slide.openClaw(); }

            if(gamepad1.dpad_up){ slide.hang(); }
            if(gamepad1.dpad_down){ slide.grab(); }
            */

//            slide.big(-gamepad1.left_stick_y);
//            slide.swing.setPosition(-gamepad1.right_stick_y);
            if(gamepad1.dpad_left){ slide.closeClaw(); }
            if(gamepad1.dpad_right){ slide.openClaw(); }


            //slide.slide();

            telemetry.addData("arm1", slide.arm1.getPosition());
            telemetry.addData("arm2", slide.arm2.getPosition());
            telemetry.addData("claw", slide.claw.getPosition());
            telemetry.addData("swing", slide.swing.getPosition());



            //telemetry.addData("slide target",slide.distance);
            //telemetry.addData("slide reference",slide.ref);
            telemetry.update();
        }
    }
}
