package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    HardwareMap hardwareMap;
    public Limelight3A limelight;
    public void cameraInit(HardwareMap map){
        hardwareMap = map;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public void cameraStart(){
        limelight.start();
    }

}