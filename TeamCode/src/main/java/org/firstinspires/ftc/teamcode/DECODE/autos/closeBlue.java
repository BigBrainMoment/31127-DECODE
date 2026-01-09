package org.firstinspires.ftc.teamcode.DECODE.autos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
@Disabled
public class closeBlue extends OpMode {

    DcMotorEx FL, FR, BL, BR;
    CRServo ls, rs;

    public static float targetV = 500;
    double kP = 0.1, kV = 0.00043;
    double error;


    private static ElapsedTime timer = new ElapsedTime();




    DcMotorEx flywheel;


    @Override
    public void init(){

        ls = hardwareMap.get(CRServo.class,"leftServo");
        rs = hardwareMap.get(CRServo.class,"rightServo");


        FL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        FR = hardwareMap.get(DcMotorEx.class, "frontRight");
        BL = hardwareMap.get(DcMotorEx.class, "backLeft");
        BR = hardwareMap.get(DcMotorEx.class, "backRight");

        flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        ls.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Override
    public void start(){



        error = targetV - flywheel.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);

//        Run
        timer.reset();
        FL.setPower(0.5);
        FR.setPower(0.5);
        BL.setPower(0.5);
        BR.setPower(0.5);

        while (timer.milliseconds() <= 1250){}

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);




        // start counting


        for (int i = 0; i <= 3; i++) {
            timer.reset();
            while (timer.milliseconds() <= 1000) {}

            ls.setPower(0.5);
            rs.setPower(0.5);

            timer.reset();
            while (timer.milliseconds() <= 400) {}

            // if you got lost we still inside the for loop
            // and we are now turing off the pushing into the flywheel
            // and recounting the time
            ls.setPower(0);
            rs.setPower(0);
            // reset timer for the backup
            timer.reset();
            FL.setPower(-.2);
            FR.setPower(-.2);
            BL.setPower(-.2);
            BR.setPower(-.2);

            while (timer.milliseconds() <= 400){}

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }


        timer.reset();
        FL.setPower(-0.5);
        FR.setPower(0.5);
        BL.setPower(0.5);
        BR.setPower(-0.5);

        while (timer.milliseconds() <= 500){}

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);



    }


    @Override
    public void loop() {
//        error = targetV - flywheel.getVelocity();
        error = targetV - flywheel.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);

        telemetry.addData("targetV", targetV);
        telemetry.addData("actual velocity", flywheel.getVelocity());
        telemetry.update();


    }

    @Override
    public void stop() {
        return;
    }

}