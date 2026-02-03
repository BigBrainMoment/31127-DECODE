package org.firstinspires.ftc.teamcode.DECODE.teleops;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.control.PIDFController;

@Autonomous
public class teleopTest extends OpMode {

    public static float targetV = 500;
    double kP = 0.1, kV = 0.00043;
    double error;

    boolean headingLock;
    double botHeading;

    DcMotorEx flywheel;
    DcMotor FL, FR, BL, BR;
    PIDFController controller = new PIDFController(new PIDFCoefficients(0.1,0,0.006,0.000004));

    @Override
    public void init() {

        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    @Override
    public void init_loop() {}

    @Override
    public void start() {
        follower.getPose();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */

    @Override
    public void loop() {
        follower.update();
        draw();
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
        /// telemetry?
















        error = targetV - flywheel.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);

        telemetry.addData("targetV", targetV);
        telemetry.addData("actual velocity", flywheel.getVelocity());
        telemetry.update();
    }



    @Override
    public void stop() {}

    public void OnUpdate(){
        if (headingLock)
            follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, controller.run(), true);
    }
}
