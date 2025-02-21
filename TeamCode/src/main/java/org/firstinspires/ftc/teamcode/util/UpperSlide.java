package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


public class UpperSlide {
    HardwareMap hardwareMap;
    public ServoImplEx arm, swing, claw;
    PwmControl.PwmRange swingRange = new PwmControl.PwmRange(500, 2500);
    PwmControl.PwmRange armRange = new PwmControl.PwmRange(500, 2500);
    PwmControl.PwmRange clawRange = new PwmControl.PwmRange(500, 1150);
    double Kp = PIDConstant.Kp;
    double Ki = PIDConstant.Ki;
    double Kd = PIDConstant.Kd;
    double lastError;
    ElapsedTime timer = new ElapsedTime();
    double integralSum = 0;
    static final double     PI=3.14;
    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 40.0 * PI;
    static final double     DRIVE_GEAR_REDUCTION    = 18.88;
    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_CM           = (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM)*10;
    public double distance = 0;
    public double ref = 0;
    DcMotor slide1, slide2;
    public void initialize(HardwareMap map) {
        hardwareMap = map;
        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");

        arm = hardwareMap.get(ServoImplEx.class, "swervo");
        arm.setDirection(ServoImplEx.Direction.REVERSE);
        arm.setPwmRange(armRange);
        claw = hardwareMap.get(ServoImplEx.class, "servo");
        claw.setPwmRange(clawRange);
        swing = hardwareMap.get(ServoImplEx.class, "swing");
        arm.setDirection(ServoImplEx.Direction.REVERSE);
        swing.setPwmRange(swingRange);

        slide1.setDirection(DcMotor.Direction.REVERSE);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void pos0(){ distance = 0; }
    public void pos1(){
        closeClaw();
        distance = Math.round(COUNTS_PER_CM*20);
        hang();
    }
    public void pos2(){ distance = Math.round(COUNTS_PER_CM*50); }
    public void pos3(){ distance = Math.round(COUNTS_PER_CM*71); }

    public void preset(){

    }

    public void grab(){
        arm.setPosition(0);
    }

    public void pause(){
        arm.setPosition(0.25);
    }
    public void hang(){
        arm.setPosition(0.45);
    }

    public void transfer(){ arm.setPosition(0.5);}

    public void out(double val){
        swing.setPosition(-val+1);
    }


    public void openClaw(){ claw.setPosition(1); }
    public void closeClaw(){ claw.setPosition(0); }
    public void slide(){
        ref = (slide1.getCurrentPosition() + slide2.getCurrentPosition()) >> 1;
        double power = PID(distance,ref);
        slide1.setPower(power);
        slide2.setPower(power);
    }
    public double PID(double refrence, double state) {
        double error = refrence - state;
        integralSum += error * timer.seconds(); //
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }
}
