package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Autonomous(group="Test")
public class TestUpperSlide228 extends LinearOpMode {
    public static double Kp = 0.01;
    public static double Ki = 0;
    public static double Kd = 0;
    static final double     PI=3.14;
    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 40.0 * PI;
    static final double     DRIVE_GEAR_REDUCTION    = 15.17;
    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     C                       = (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM)*10;
    public static double targetX = C*30;
    public static DcMotor motorL, motorR;
    public static double pL, pR;
    public static double X = 0, lastX = 0;
    public static ElapsedTime timer;
    public static double integralX;

    public void initialize(){
        timer = new ElapsedTime();
        integralX = 0;
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        motorR = hardwareMap.get(DcMotor.class, "slide1");
        motorL = hardwareMap.get(DcMotor.class, "slide2");
        motorR.setDirection(DcMotor.Direction.FORWARD);
        motorL.setDirection(DcMotor.Direction.REVERSE);
    }
    public void resetmotor(){
        X = 0;
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double pid() {
        double deltaX = targetX - X;
        telemetry.addData("error", deltaX);
        double derivativeX = (X - lastX)/timer.seconds();
        lastX = X;
        integralX += deltaX * timer.seconds();
        timer.reset();
        return deltaX * Kp + derivativeX * Kd + integralX * Ki;
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        resetmotor();
        while(opModeIsActive()){
            pL = motorL.getCurrentPosition();
            pR = motorR.getCurrentPosition();
            X = (pL + pR) / 2;
            targetX = C*30*gamepad1.left_stick_x + 10;
            double power = pid();
            motorL.setPower(power);
            motorR.setPower(power);
            telemetry.addData("Left Motor Position", pL);
            telemetry.addData("Right Motor Position", pR);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}
