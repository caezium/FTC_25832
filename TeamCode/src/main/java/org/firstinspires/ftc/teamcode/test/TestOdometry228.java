package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Localizer2802;
import org.firstinspires.ftc.teamcode.util.Datalogger;

import java.util.List;

@TeleOp(group="TeleOp")
public class TestOdometry228 extends LinearOpMode {
    Localizer2802 odo = new Localizer2802();

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        odo.initialize(hardwareMap);

        ElapsedTime elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        waitForStart();
        odo.resetEncoder();
        while (opModeIsActive()) {
            if (gamepad1.circle) {
                odo.resetEncoder();
            }

            double botHeading = Localizer2802.theta;
            odo.positionArc();

            telemetry.addData("Status", "Running");

            telemetry.addData("X", Localizer2802.X);
            telemetry.addData("Y", Localizer2802.Y);
            telemetry.addData("Heading", Localizer2802.angleWrap(Localizer2802.theta));

            telemetry.addData("Recording Rate (Hz)", 1 / elapsedtime.seconds());
            telemetry.addData("Loop Time (ms)", elapsedtime.milliseconds());
            telemetry.update();
            elapsedtime.reset();

        }
    }
}
