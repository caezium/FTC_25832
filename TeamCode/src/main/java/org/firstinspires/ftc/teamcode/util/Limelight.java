package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    HardwareMap hardwareMap;
    public Limelight3A limelight;
    public void initialize(HardwareMap map){
        hardwareMap = map;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.resetDeviceConfigurationForOpMode();

    }

    public void cameraStart(){
        limelight.start();
        limelight.reloadPipeline();
    }

    public double getAngle(){
        limelight.reloadPipeline();
        limelight.captureSnapshot("snapfinal");
        return limelight.getLatestResult().getPythonOutput()[1] * 5;
    }

}