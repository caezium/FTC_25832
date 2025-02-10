package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class LowerSlide {
    HardwareMap hardwareMap;
    ServoImplEx part1, part2, part3, slide1, slide2, spinclaw, claw;

    PwmControl.PwmRange v4range = new PwmControl.PwmRange(500, 2500);
    PwmControl.PwmRange downrange = new PwmControl.PwmRange(500, 900);

    PwmControl.PwmRange clawRange = new PwmControl.PwmRange(500, 1150);

    public void initialize(HardwareMap map) {
        hardwareMap = map;

        part1 = hardwareMap.get(ServoImplEx.class, "apple");
        part2 = hardwareMap.get(ServoImplEx.class, "bob");
        part3 = hardwareMap.get(ServoImplEx.class, "cyliis");

        slide1 = hardwareMap.get(ServoImplEx.class, "down1");
        slide2 = hardwareMap.get(ServoImplEx.class, "down2");

        spinclaw = hardwareMap.get(ServoImplEx.class, "spinclaw");
        claw = hardwareMap.get(ServoImplEx.class, "claw");

        part2.setDirection(ServoImplEx.Direction.REVERSE);
        part1.setPwmRange(v4range);
        part2.setPwmRange(v4range);
        part3.setPwmRange(v4range);
        spinclaw.setPwmRange(v4range);
        claw.setPwmRange(clawRange);

        slide2.setDirection(ServoImplEx.Direction.REVERSE);
        slide1.setPwmRange(downrange);
        slide2.setPwmRange(downrange);
    }

    public void low(double val){
        slide1.setPosition(val);
        slide2.setPosition(val);
    }

    public void bigarm(double val){
        part1.setPosition(val);
        part2.setPosition(val);
    }

    public void smallclaw(double val){ part3.setPosition(val); }

    public void pos0(){ spinclaw.setPosition(1);}
    public void pos1(){ spinclaw.setPosition(0.25);}
    public void pos2(){ spinclaw.setPosition(0.5);}
    public void pos3(){ spinclaw.setPosition(0.75);}
    //public void pos4(){ spinclaw.setPosition(0);}

    public void closeclaw(){
        claw.setPosition(1);
    }

    public void openclaw(){
        claw.setPosition(0);
    }

}
