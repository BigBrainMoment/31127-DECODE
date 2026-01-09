package org.firstinspires.ftc.teamcode.DECODE.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled
public class flyPID extends LinearOpMode {

    DcMotorEx flywheel;
    //Servo flicky;

    public static float targetV = 0;

    double kP = 0.1, kV = 0.00043;
    double error;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        waitForStart();

        while(opModeIsActive()) {


            if (gamepad1.a) {
                targetV = 1000;
            }
            if (gamepad1.b) {
                targetV = 1500;
            }

            error = targetV - flywheel.getVelocity();

            flywheel.setPower(kP * error + kV * targetV);

            telemetry.addData("targetV", targetV);
            telemetry.addData("actual velocity", flywheel.getVelocity());
            telemetry.update();

        }
    }
}
