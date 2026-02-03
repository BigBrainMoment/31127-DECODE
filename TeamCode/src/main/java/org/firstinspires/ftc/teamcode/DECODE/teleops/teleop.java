package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * THE REAL TELEOP
 */
@TeleOp
public class teleop extends LinearOpMode {
    int count = 0; // helpful; no more holding down the shooting button
    double setSpeed = 1; // Controls for Driving speed
    boolean run = false; // Fail safe, can not push when flywheel is off
    boolean far = false, near = false; // Shooting farther

    // Variables for the flywheel PID
    public static float targetV = 0;
    double kP = 0.1, kV = 0.00043;
    double error;


    private static ElapsedTime timer = new ElapsedTime();


    DcMotorEx flywheel;

    @Override
    public void runOpMode() throws InterruptedException{
        CRServo ls, rs;
        ls = hardwareMap.get(CRServo.class,"leftServo");
        rs = hardwareMap.get(CRServo.class,"rightServo");

        DcMotor FL, FR, BL, BR, fly;
        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");
/// CHECK WHAT WORKS
        fly = hardwareMap.get(DcMotor.class,"flywheel");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        fly.setDirection(DcMotorSimple.Direction.REVERSE);
        ls.setDirection(DcMotorSimple.Direction.REVERSE);



        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");




        waitForStart();
        if (isStopRequested()) return;


        while(opModeIsActive()) {

            rs.setPower(-.67);
            ls.setPower(-.67);
            // Calculations for drive train
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);


            double frontLeftPower = 0.5 * ((y + x + rx) / denominator);
            double backLeftPower = 0.5 * ((y - x + rx) / denominator);
            double frontRightPower = 0.5 * ((y - x - rx) / denominator);
            double backRightPower = 0.5 * ((y + x - rx) / denominator);

            //teleop goes on here  -- where you tell the robot exactly what to do during the program
            FL.setPower(frontLeftPower * setSpeed);
            BL.setPower(backLeftPower * setSpeed);
            FR.setPower(frontRightPower * setSpeed);
            BR.setPower(backRightPower * setSpeed);


/// REMIND KENNY THIS CHANGED
            //speed settings
            if (gamepad1.dpad_up) {
                setSpeed = .3;
            }
            if (gamepad1.dpad_right) {
                setSpeed = 1;
            }
            if (gamepad1.dpad_down) {
                setSpeed = 1.5;
            }
            if (gamepad1.dpad_left) {
                setSpeed = 2.25;
            }

            //Servo push into flywheel
            if (gamepad2.aWasPressed() && run) {
                rs.setPower(.67);
                ls.setPower(.67);
                timer.reset();
                while (timer.milliseconds() <= 400){}
                rs.setPower(-.67);
                ls.setPower(-.67);
            }




            // Counter for far shooting
            if (gamepad2.dpad_left){
                near = true;
            } else if (gamepad2.dpadLeftWasReleased()){
                near = false;
            }

            if (gamepad2.dpad_right){
                far = true;
            } else if (gamepad2.dpadRightWasReleased()){
                far = false;
            }

            // Counter for left bumper
            if (gamepad2.leftBumperWasPressed()) {
                count++;
            }
            // The Flywheel settings for left bumper
            if (count % 2 != 0) {
                targetV = 0;
                run = false;
            } else if (count == 0) {
                targetV = 0;
                run = false;
            } else if (far){
/// TUNE THIS VALUE TO THE SPEED OF FAR SHOOT
                targetV = 1800;


            } else if (near){
                targetV = 1000;
            }else {
/// TUNE THIS VALUE TO THE SPEED OF CLOSE SHOOT
                targetV = 1350;


                while (timer.milliseconds() <= 500){

                }
                run = true;
            }






            // Flywheel PID calculation and code that also makes speed of motor appear
            error = targetV - flywheel.getVelocity();

            fly.setPower(kP * error + kV * targetV);

            telemetry.addData("targetV", targetV);
            telemetry.addData("targetV", far);
            telemetry.addData("actual velocity", flywheel.getVelocity());
            telemetry.update();

        }
    }
}
