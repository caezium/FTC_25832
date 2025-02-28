package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Localizer2802;
import org.firstinspires.ftc.teamcode.util.Datalogger;

import java.util.List;

@TeleOp(group="TeleOp")
public class TestEncoder extends LinearOpMode {
        Localizer odo = new Localizer2802();
        Drivetrain drive = new Drivetrain();

        @Override
        public void runOpMode() throws InterruptedException {
                List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
                for (LynxModule hub : allHubs) {
                        hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
                }
                odo.initialize(hardwareMap);
                drive.initialize(hardwareMap);

                ElapsedTime elapsedtime = new ElapsedTime();
                elapsedtime.reset();

                waitForStart();

                while (opModeIsActive()) {
                        if (gamepad1.circle) {
                                odo.resetEncoder();
                        }
                      
                        double botHeading = Localizer.theta;
                        odo.positionArc();

                        telemetry.addData("Status", "Running");

                        telemetry.addData("X", Localizer.X);
                        telemetry.addData("Y", Localizer.Y);
                        telemetry.addData("Heading", Localizer.angleWrap(Localizer.theta));

                        telemetry.addData("Hertz", 1 / elapsedtime.seconds());
                        telemetry.addData("Loop Time", elapsedtime.seconds());
                        telemetry.update();
                        elapsedtime.reset();

                }
        }
}
