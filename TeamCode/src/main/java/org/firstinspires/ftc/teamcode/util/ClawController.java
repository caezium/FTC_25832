package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ClawController {
        private ServoImplEx clawServo;
        private final Limelight vision;

        // Control constants
        private static final double ROTATION_INCREMENT = 0.02; // Amount to rotate per update
        private static final double CURRENT_THRESHOLD = 1.0; // Amps, threshold for detecting grip
        private static final int MAX_ROTATION_ATTEMPTS = 50; // Prevent infinite rotation

        // Servo positions
        private static final double MIN_POSITION = 0.0;
        private static final double MAX_POSITION = 1.0;
        private double currentPosition = 0.5; // Start at center position

        public ClawController(HardwareMap hardwareMap, String servoName) {
                vision = new Limelight();
                vision.cameraInit(hardwareMap);

                // Initialize servo
                clawServo = hardwareMap.get(ServoImplEx.class, servoName);
                clawServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
                clawServo.setPosition(currentPosition);
        }

        /**
         * Start vision processing
         */
        public void start() {
                vision.cameraStart();
        }

        /**
         * Update claw rotation based on vision feedback
         * 
         * @return true if aligned with sample
         */
        public boolean updateRotation() {
                if (!vision.isSampleDetected()) {
                        return false;
                }

                double angleOffset = vision.getSampleAngleOffset();

                // Calculate rotation direction and amount
                double rotationAmount = Math.signum(angleOffset) * ROTATION_INCREMENT;

                // Update position within bounds
                currentPosition = Math.min(MAX_POSITION,
                                Math.max(MIN_POSITION,
                                                currentPosition + rotationAmount));

                clawServo.setPosition(currentPosition);

                return vision.isAlignedWithSample();
        }

        /**
         * Check if the claw has gripped a sample based on current draw
         * 
         * @return true if sample is gripped
         */
        public boolean isGripping() {
                return clawServo.getCurrentDraw() >= CURRENT_THRESHOLD;
        }

        /**
         * Attempt to auto-align with and grip a sample
         * 
         * @return true if sample was successfully gripped
         */
        public boolean autoGripSample() {
                int attempts = 0;

                while (attempts < MAX_ROTATION_ATTEMPTS) {
                        if (updateRotation()) {
                                // We're aligned, check if we've gripped
                                if (isGripping()) {
                                        return true;
                                }
                        }
                        attempts++;

                        try {
                                Thread.sleep(50); // Brief pause between updates
                        } catch (InterruptedException e) {
                                Thread.currentThread().interrupt();
                                return false;
                        }
                }

                return false;
        }

        /**
         * Get current servo position
         * 
         * @return position from 0.0 to 1.0
         */
        public double getCurrentPosition() {
                return currentPosition;
        }

        /**
         * Set servo position directly
         * 
         * @param position desired position from 0.0 to 1.0
         */
        public void setPosition(double position) {
                currentPosition = Math.min(MAX_POSITION, Math.max(MIN_POSITION, position));
                clawServo.setPosition(currentPosition);
        }
}
