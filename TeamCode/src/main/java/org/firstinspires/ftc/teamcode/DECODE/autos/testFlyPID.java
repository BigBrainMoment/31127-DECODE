package org.firstinspires.ftc.teamcode.DECODE.autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "tests")
public class testFlyPID extends OpMode {
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static float targetV = 500;
    double kP = 0.1, kV = 0.00043;
    double error;
    private int pathState;
    private static ElapsedTime timer = new ElapsedTime();

    DcMotorEx flywheel;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    CRServo ls, rs;

    public void pause(){
        while (timer.milliseconds() <= 400){

        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
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
                break;

            case 1:

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



                timer.reset();
                FL.setPower(0.5);
                FR.setPower(-0.5);
                BL.setPower(-0.5);
                BR.setPower(0.5);

                while (timer.milliseconds() <= 500){

                }

                FL.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                BR.setPower(0);
        }
    }

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class,"flywheel");


        ls = hardwareMap.get(CRServo.class,"leftServo");
        rs = hardwareMap.get(CRServo.class,"rightServo");

        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");


        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        ls.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        targetV = 1000;
        setPathState(0);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        pause();
        autonomousPathUpdate();
        error = targetV - flywheel.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);

        telemetry.addData("targetV", targetV);
        telemetry.addData("actual velocity", flywheel.getVelocity());
        telemetry.update();
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}