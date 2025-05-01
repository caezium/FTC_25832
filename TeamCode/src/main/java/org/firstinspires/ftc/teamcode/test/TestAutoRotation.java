package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.ClawController;

@Autonomous(group = "Test")
public class TestAutoRotation extends LinearOpMode {
        private ClawController clawController;
        private static final String CLAW_SERVO_NAME = "claw"; // Update to match your config

        @Override
        public void runOpMode() throws InterruptedException {
                // Initialize claw controller
                clawController = new ClawController(hardwareMap, CLAW_SERVO_NAME);

                telemetry.addData("Status", "Initialized");
                telemetry.update();

                waitForStart();
                clawController.start(); // Start vision processing

                while (opModeIsActive()) {
                        if (gamepad1.a) {
                                // Attempt auto-grip when A is pressed
                                telemetry.addData("Status", "Starting auto-grip sequence");
                                telemetry.update();

                                boolean success = clawController.autoGripSample();

                                telemetry.addData("Auto-grip result", success ? "Success!" : "Failed");
                        }

                        // Manual control with bumpers (for testing/backup)
                        if (gamepad1.right_bumper) {
                                clawController.setPosition(clawController.getCurrentPosition() + 0.05);
                        } else if (gamepad1.left_bumper) {
                                clawController.setPosition(clawController.getCurrentPosition() - 0.05);
                        }

                        // Display status
                        telemetry.addData("Sample Detected", clawController.vision.isSampleDetected());
                        if (clawController.vision.isSampleDetected()) {
                                telemetry.addData("Angle Offset", "%.2f°",
                                                clawController.vision.getSampleAngleOffset());
                                telemetry.addData("Aligned", clawController.vision.isAlignedWithSample());
                        }
                        telemetry.addData("Servo Position", "%.2f", clawController.getCurrentPosition());
                        telemetry.addData("Gripping", clawController.isGripping());
                        telemetry.addData("Controls", "A = Auto-grip, Bumpers = Manual adjust");
                        telemetry.update();

                        sleep(50); // Brief pause between updates
                }
        }
}
