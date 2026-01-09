package org.firstinspires.ftc.teamcode.DECODE.autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class farRed extends OpMode {
    DcMotor FL, FR, BL, BR;
    CRServo ls, rs;
    DcMotorEx flywheel;
    // The variables for PID for flywheel
    public static float targetV = 0;
    double kP = 0.1, kV = 0.00043;
    double error;


    private static ElapsedTime timer = new ElapsedTime();
    public void drive(double milliseconds, int power, int leftNegative, int rightNegative) {
        timer.reset();
        FL.setPower(power * leftNegative);
        FR.setPower(power * rightNegative);
        BL.setPower(power * rightNegative);
        BR.setPower(power * leftNegative);

        while (timer.milliseconds() <= milliseconds){}

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }



    public void push(int artifacts){
        timer.reset();
        ls.setPower(0);
        rs.setPower(0);

        while (timer.milliseconds() <= 400 * artifacts){}

        ls.setPower(0);
        rs.setPower(0);
    }




    public void run(){
        CRServo ls, rs;
        ls = hardwareMap.get(CRServo.class,"leftServo");
        rs = hardwareMap.get(CRServo.class,"rightServo");

        DcMotor FL, FR, BL, BR;

        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");

        flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        ls.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /// CODE THE THING HERE
        drive(500,1,1,1);
        push(3);

    }



    @Override
    public void init(){
        run();
    }

    @Override
    public void loop(){
        error = targetV - flywheel.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);

        telemetry.addData("targetV", targetV);
        telemetry.addData("actual velocity", flywheel.getVelocity());
        telemetry.update();
    }
}

