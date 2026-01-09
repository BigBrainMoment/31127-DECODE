package org.firstinspires.ftc.teamcode.DECODE.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TESTred", group = "Examples")
@Disabled
public class closeRed extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    DcMotorEx FL, FR, BL, BR;
    CRServo ls, rs;
    DcMotorEx flywheel;
    public static float targetV = 500;
    double kP = 0.1, kV = 0.00043;
    double error;


    private static ElapsedTime timer = new ElapsedTime();

    public void autonomousPathUpdate() {

        targetV = 800;
        while (timer.milliseconds() <= 1250){}

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

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        error = targetV - flywheel.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);

        telemetry.addData("targetV", targetV);
        telemetry.addData("actual velocity", flywheel.getVelocity());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        error = targetV - flywheel.getVelocity();

        flywheel.setPower(kP * error + kV * targetV);

        telemetry.addData("targetV", targetV);
        telemetry.addData("actual velocity", flywheel.getVelocity());
        telemetry.update();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}


}