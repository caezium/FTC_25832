package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.UpperSlide;

@TeleOp(group="Test")
public class TestUpperSlide extends LinearOpMode {

    UpperSlide slide = new UpperSlide();
    @Override
    public void runOpMode() {
        slide.initialize(hardwareMap);
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.cross){ slide.pos0(); }
            if(gamepad1.circle){ slide.pos1(); }
            if(gamepad1.triangle){ slide.pos2(); }
            if(gamepad1.square){ slide.pos3(); }


            if(gamepad1.dpad_left){ slide.closeClaw(); }
            if(gamepad1.dpad_right){ slide.openClaw(); }

            if(gamepad1.dpad_up){ slide.hang(); }
            if(gamepad1.dpad_down){ slide.grab(); }

            slide.swing.setPosition(gamepad1.left_stick_x);

            slide.slide();

            telemetry.addData("arm", slide.arm.getPosition());
            telemetry.addData("claw", slide.claw.getPosition());
            telemetry.addData("swing", slide.swing.getPosition());

            telemetry.addData("slide target",slide.distance);
            telemetry.addData("slide reference",slide.ref);
            telemetry.update();
        }
    }
}
