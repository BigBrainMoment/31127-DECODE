package org.firstinspires.ftc.teamcode.DECODE.teleops;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class Drivetrain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor FL, FR, BL, BR;
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
/// CHECK WHAT WORKS


        waitForStart();
        if (isStopRequested()) return;


        while(opModeIsActive()) {

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
            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

        }
    }
}

