package org.firstinspires.ftc.teamcode.DECODE.autos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/// THIS ONE IS THE WORKING FAR AUTO
@Autonomous (name="farRED", group="Linear OpMode")
public class runRed extends LinearOpMode {
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;


    private static ElapsedTime timer = new ElapsedTime();


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
/**
 * HERE IS WHERE YOU CHANGE MOTOR SPEED
 */
            fly.setPower(0.87);
            while (timer.milliseconds() <= 3000){

            }
//        Run
            // start counting


            for (int i = 0; i <= 3; i++) {
                timer.reset();
                while (timer.milliseconds() <= 2000) {

                }

                ls.setPower(0.5);
                rs.setPower(0.5);

                timer.reset();
                while (timer.milliseconds() <= 300) {

                }

                // if you got lost we still inside the for loop
                // and we are now turing off the pushing into the flywheel
                // and recounting the time
                ls.setPower(0);
                rs.setPower(0);
            }

            while (timer.milliseconds() <= 1000){

            }

            timer.reset();
            FL.setPower(-0.5);
            FR.setPower(-0.5);
            BL.setPower(-0.5);
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
