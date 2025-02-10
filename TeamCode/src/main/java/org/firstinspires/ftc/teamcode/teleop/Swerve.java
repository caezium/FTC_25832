package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Camera;
import org.firstinspires.ftc.teamcode.util.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.LowerSlide;
import org.firstinspires.ftc.teamcode.util.UpperSlide;

@TeleOp(group="TeleOp")
public class Swerve extends LinearOpMode {
    Localizer odo = new Localizer();
    Drivetrain drive = new Drivetrain();
    UpperSlide upslide = new UpperSlide();
    LowerSlide lowslide = new LowerSlide();
    //Camera camera = new Camera();



    @Override
    public void runOpMode() throws InterruptedException {
        odo.initialize(hardwareMap);
        drive.initialize(hardwareMap);
        upslide.initialize(hardwareMap);
        lowslide.initialize(hardwareMap);
        //camera.cameraStart(hardwareMap);
        lowslide.openclaw();
        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.cross){ upslide.pos0(); }
            if(gamepad1.circle){ upslide.pos1(); }
            if(gamepad1.triangle){ upslide.pos2(); }
            if(gamepad1.square){ upslide.pos3(); }

            if(gamepad1.dpad_left){ upslide.closeClaw(); }
            if(gamepad1.dpad_right){ upslide.openClaw(); }

            if(gamepad1.dpad_up){
                upslide.pause();
                sleep(1000);
                upslide.hang();
            }
            if(gamepad1.dpad_down){
                upslide.pause();
                sleep(1000);
                upslide.grab();
            }

            if(gamepad1.left_bumper){ odo.resetIMU();}

            if(gamepad2.a){ lowslide.pos0(); }
            if(gamepad2.x){ lowslide.pos1(); }
            if(gamepad2.y){ lowslide.pos2(); }
            if(gamepad2.b){ lowslide.pos3(); }

            if(gamepad2.right_bumper) {
                lowslide.bigarm(1);
                lowslide.smallclaw(1);
            }
            if(gamepad2.left_bumper){
                lowslide.bigarm(0.8 );
                lowslide.smallclaw(0);
                sleep(500);
                lowslide.closeclaw();
            }

            if(gamepad2.dpad_right){ lowslide.openclaw();}
            if(gamepad2.dpad_left){ lowslide.closeclaw();}

            upslide.out(gamepad1.left_trigger);
            lowslide.low(gamepad2.left_stick_x);

            //lowslide.low(gamepad2.left_stick_x);
            //lowslide.bigarm(gamepad2.left_trigger);
            //lowslide.smallclaw(gamepad2.right_trigger);
            //telemetry.addData("bigarm",gamepad2.left_trigger);
            //telemetry.addData("small",gamepad2.right_trigger);

            upslide.slide();


            //camera.clawTurn();
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
            telemetry.addData("claw", upslide.claw.getPosition());
            telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Heading", botHeading);
            telemetry.addData("Status", "Running");

            telemetry.update();
        }
    }
}