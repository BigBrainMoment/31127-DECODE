package org.firstinspires.ftc.teamcode.DECODE.autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "testPedro")
public class testSisterPedro extends OpMode {
    DcMotorEx FL, FR, BL, BR;

    GoBildaPinpointDriver pinpoint;

    private int pathState;


    private Timer pathTimer, actionTimer, opmodeTimer;
    private final Pose startPose = new Pose(63.5, 8, Math.toRadians(90));

    private final Pose scorePose = new Pose(60, 14, Math.toRadians(111.2)); //figure outt
    private final Pose rescorePose = new Pose(59.75, 13.75, Math.toRadians(111.2)); //figure outt 111.75
    private final Pose prescorePose = new Pose(50.5, 20, Math.toRadians(180)); //figure outt
    private final Pose pickup1Pose = new Pose(18.67, 36.83, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose control = new Pose( 55.15, 42.81, Math.toRadians(180)); // Scoring Pose 2 of our robot. goes forward to intake
    private final Pose secondcontrol = new Pose(80, 59, Math.toRadians(180)); // Scoring Pose 2 of our robot. goes forward to intake

    private final Pose pickup2Pose = new Pose(23, 58, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(56.5, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose finishPose = new Pose(50.5, 25.0, Math.toRadians(108.0));

    private PathChain grabPickup1, return21, intake1, return1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, startshoot, return11, actuallyscorePickup2, park;
    private Path grab1;

    public void buildPaths() {
        startshoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

//        grab1 = new Path(new BezierLine(scorePose, pickup1Pose));
//        grab1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, control, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1Pose.getHeading())
                .setTValueConstraint(0.5)
                .build();

        return1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, control, rescorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), rescorePose.getHeading())
                .build();

        return11 = follower.pathBuilder()
                .addPath(new BezierLine(prescorePose, scorePose))
                .setLinearHeadingInterpolation(prescorePose.getHeading(), scorePose.getHeading())
                .build();
        return21 = follower.pathBuilder()
                .addPath(new BezierLine(prescorePose, rescorePose))
                .setLinearHeadingInterpolation(prescorePose.getHeading(), rescorePose.getHeading())
                .build();


        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, secondcontrol, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, rescorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), rescorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, finishPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), finishPose.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                follower.followPath(startshoot,true );
                setPathState(-2);

                break;
            case -2:
                if (pathTimer.getElapsedTimeSeconds()>2.5) {
                    setPathState(0);
                }
                break;
            case 0:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(1);
                }

                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds()>0.8) {
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(7);

                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds()>0.15) {
                    follower.followPath(intake1);
                    setPathState(-8);
                }

                break;

            case -8:
                if(!follower.isBusy())
                {
                    follower.followPath(return1);
                    setPathState(-10);
                }
                break;

            case -10:
                if (!follower.isBusy()) {
//                    follower.followPath(return21);
                    setPathState(9);
                }
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 1.1 && !follower.isBusy()) {
                }
                if (pathTimer.getElapsedTimeSeconds()>1.30 && !follower.isBusy()) {
                    setPathState(10);
                }

                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds()>0.1) {
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds()>0.80) {
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(16);
                }
                break;





            case 16:
                if(pathTimer.getElapsedTimeSeconds()>0.15) {

                    follower.followPath(grabPickup2, true);
                    setPathState(17);
                }
                break;

            case 17:
                if(!follower.isBusy())
                {
                    follower.followPath(return1,true);
                    setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
//                    follower.followPath(return21);
                    setPathState(19);
                }
            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 1.35 && !follower.isBusy()) {
                }
                if (pathTimer.getElapsedTimeSeconds()>1.55 && !follower.isBusy()) {
                    setPathState(20);
                }

                break;
            case 20:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(21);
                }
                break;
            case 21:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(23);
                }
                break;
            case 23:
                if (pathTimer.getElapsedTimeSeconds()>0.15) {
                    setPathState(24);
                }
                break;
            case 24:
                if (pathTimer.getElapsedTimeSeconds()>0.80) {
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTimeSeconds()>0.20) {
                    setPathState(26);
                }
                break;
            case 26:
                if (!follower.isBusy()) {
                    follower.followPath(park);
                    setPathState(67);
                }
                break;
            case 67:
                if (!follower.isBusy()) {
                    //autoendpose = follower.getPose();
                }

        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {


        FL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        FR = hardwareMap.get(DcMotorEx.class, "frontRight");
        BL = hardwareMap.get(DcMotorEx.class, "backLeft");
        BR = hardwareMap.get(DcMotorEx.class, "backRight");
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setHeading(0, AngleUnit.DEGREES);

        // Configure the sensor
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.9);


    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(-1);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

    }

    @Override
    public void stop() {
       // autoendpose = follower.getPose();
    }


}