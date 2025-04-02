package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Limelight;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.LowerSlide;
import org.firstinspires.ftc.teamcode.util.UpperSlide;

@TeleOp(group="TeleOp")
public class Swerve extends LinearOpMode {
    Localizer odo = new Localizer();
    Drivetrain drive = new Drivetrain();
    UpperSlide upslide = new UpperSlide();
    LowerSlide lowslide = new LowerSlide();
    Limelight camera = new Limelight();

    @Override
    public void runOpMode() throws InterruptedException {
        odo.initialize(hardwareMap);
        drive.initialize(hardwareMap);
        upslide.initialize(hardwareMap);
        lowslide.initialize(hardwareMap);
        camera.cameraInit(hardwareMap);



        waitForStart();

        camera.cameraStart();
        while (opModeIsActive()) {

            if(gamepad1.a){ upslide.pos0(); }
            if(gamepad1.x){ upslide.pos1(); }
            if(gamepad1.y){ upslide.pos2(); }
            if(gamepad1.b){ upslide.pos3(); }
            if(gamepad2.a){ upslide.front(); }
            if(gamepad2.b){ upslide.behind(); }

            upslide.big(gamepad1.right_trigger);
            upslide.swing.setPosition(gamepad1.left_trigger);
            if(gamepad1.left_bumper){ upslide.closeClaw(); }
            if(gamepad1.right_bumper){ upslide.openClaw(); }

            lowslide.big(-gamepad2.left_stick_y);
            lowslide.small(-gamepad2.right_stick_y);
            lowslide.spinclaw.setPosition(gamepad2.right_trigger);
            if(gamepad2.left_bumper){ lowslide.closeClaw(); }
            if(gamepad2.right_bumper){ lowslide.openClaw(); }

            upslide.slide();

            double angle = camera.limelight.getLatestResult().getPythonOutput()[1];



            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = odo.heading();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            drive.fl(frontLeftPower);
            drive.bl(backLeftPower);
            drive.fr(frontRightPower);
            drive.br(backRightPower);



            //Telemetry
            //telemetry.addData("claw", upslide.claw.getPosition());
            telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("big arm", upslide.arm1.getPosition());
            telemetry.addData("small arm", upslide.swing.getPosition());
            telemetry.addData("Heading", botHeading);
            telemetry.addData("Status", "Running");

            telemetry.update();
        }
    }
}