package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    private HardwareMap hardwareMap;
    public Limelight3A limelight;

    // Vision processing constants
    private static final int SAMPLE_PIPELINE = 0; // Pipeline index for sample detection
    private static final double MIN_CONFIDENCE = 0.5; // Minimum confidence for valid detection
    private static final double TARGET_X = 0.0; // Target x-coordinate for alignment (center)
    private static final double ALIGNMENT_THRESHOLD = 2.0; // Degrees of acceptable misalignment

    public void cameraInit(HardwareMap map) {
        hardwareMap = map;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(SAMPLE_PIPELINE);
    }

    public void cameraStart() {
        limelight.start();
    }

    /**
     * Checks if a sample is currently detected in the camera view
     * 
     * @return true if a sample is detected with sufficient confidence
     */
    public boolean isSampleDetected() {
        LLResult result = limelight.getLatestResult();
        double[] data = result.getPythonOutput();
        // Check if we have valid data and confidence exceeds threshold
        return data != null && data.length >= 1 && data[0] >= MIN_CONFIDENCE;
    }

    /**
     * Gets the angle offset to the detected sample
     * 
     * @return angle in degrees, positive means sample is to the right
     */
    public double getSampleAngleOffset() {
        LLResult result = limelight.getLatestResult();
        double[] data = result.getPythonOutput();
        if (data != null && data.length >= 2) {
            return data[1]; // Assuming data[1] contains horizontal offset in degrees
        }
        return 0.0;
    }

    /**
     * Checks if the claw is aligned with the detected sample
     * 
     * @return true if aligned within threshold
     */
    public boolean isAlignedWithSample() {
        if (!isSampleDetected())
            return false;
        double offset = Math.abs(getSampleAngleOffset() - TARGET_X);
        return offset <= ALIGNMENT_THRESHOLD;
    }

}
