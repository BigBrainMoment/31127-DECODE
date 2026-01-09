package org.firstinspires.ftc.teamcode.DECODE.autos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/// THIS ONE IS THE WORKING CLOSE AUTO
@Autonomous (name="BLUE", group="Linear OpMode")
public class nearBlue extends LinearOpMode {
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;


    private static ElapsedTime timer = new ElapsedTime();
    public void drive(double milliseconds) {
        timer.reset();
        FL.setPower(1);
        FR.setPower(1);
        BL.setPower(1);
        BR.setPower(1);

        while (timer.milliseconds() <= milliseconds){

        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo ls, rs;
        ls = hardwareMap.get(CRServo.class,"leftServo");
        rs = hardwareMap.get(CRServo.class,"rightServo");

        DcMotor FL, FR, BL, BR, fly;
        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");

        fly = hardwareMap.get(DcMotor.class,"flywheel");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        ls.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        waitForStart();
        if (isStopRequested()) return;

        ElapsedTime myTimer = new ElapsedTime();
        if (opModeIsActive()) {
            fly.setPower(.54);

//        Run
            timer.reset();
            FL.setPower(0.5);
            FR.setPower(0.5);
            BL.setPower(0.5);
            BR.setPower(0.5);

            while (timer.milliseconds() <= 1250){

            }

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);




            // start counting


            for (int i = 0; i <= 3; i++) {
                timer.reset();
                while (timer.milliseconds() <= 1200) {

                }

                ls.setPower(0.5);
                rs.setPower(0.5);

                timer.reset();
                while (timer.milliseconds() <= 400) {

                }

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

                while (timer.milliseconds() <= 400){

                }

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

            while (timer.milliseconds() <= 500){

            }

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

        }


    }




}