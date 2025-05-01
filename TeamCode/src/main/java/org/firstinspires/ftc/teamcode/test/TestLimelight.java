package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Limelight;

@Autonomous(group="Test")
public class TestLimelight extends LinearOpMode {

    Limelight camera = new Limelight();

    @Override
    public void runOpMode() throws InterruptedException {
        camera.initialize(hardwareMap);


        waitForStart();
        camera.cameraStart();
        while(opModeIsActive()) {

            LLStatus status = camera.limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

//            LLResult result = camera.limelight.getLatestResult();
//
//            telemetry.addData("result", java.util.Arrays.toString(result.getPythonOutput()));
            telemetry.addData("Angle", camera.getAngle());
            telemetry.addData("isRunning", camera.limelight.isRunning());
            telemetry.addData("Python Output", java.util.Arrays.toString(camera.limelight.getLatestResult().getPythonOutput()));
            telemetry.update();
        }
    }
}
