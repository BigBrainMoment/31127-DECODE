package org.firstinspires.ftc.teamcode.DECODE.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;


@TeleOp(name = "t")
@Configurable
public class test extends NextFTCOpMode {
    private static ElapsedTime timer = new ElapsedTime();
    int count = 0; // helpful; no more holding down the shooting button
    double setSpeed = 1; // Controls for Driving speed
    boolean run = false; // Fail safe, can not push when flywheel is off
    boolean far = false, near = false; // Shooting farther


    private Follower follower;
    private PathChain pathChain;
    private TelemetryManager telemetryM;


    private boolean slowMode = false;
    double headinglockangle;
    double posx, posy, distx, disty;
    double diagonaldist;
    double trigangle;

    // Variables for the flywheel PID
    public static double targetV = 0;
    double kP = 0.1, kV = 0.00043; double error;
    boolean holdshooting = false;


    double turnerror;




    boolean headingLock;

    DcMotorEx FL, FR, BL, BR, flywheel;
    CRServo ls, rs;
    PIDFController controller = new PIDFController(new PIDFCoefficients(0.1,0,0.006,0.000004));
    double botHeading;



    @Override
    public void onInit() {

        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");

        FL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        BR = hardwareMap.get(DcMotorEx.class, "frontRight");
        BL = hardwareMap.get(DcMotorEx.class, "backLeft");
        FR = hardwareMap.get(DcMotorEx.class, "backRight");
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ls = hardwareMap.get(CRServo.class, "ls");
        rs = hardwareMap.get(CRServo.class, "rs");


        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(50.5, 25.0, Math.toRadians(108.0)).mirror());
        ////////////////////////follower.setStartingPose(autoendpose);
        follower.update();
        headingLock = false;

        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, new Pose(42,35)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(270), 0.8))
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void onUpdate() {
        follower.update();

        botHeading = follower.getHeading();
        posx = follower.getPose().mirror().getX();
        posy = follower.getPose().mirror().getY();
        distx = posx - 9;
        disty = Math.abs(137 - posy);
        diagonaldist = Math.sqrt(distx*distx + disty*disty);
        trigangle = Math.toDegrees(Math.atan(disty/distx));
        headinglockangle = trigangle;





        if (gamepad1.b) {
            targetV = 0;
        } else {
            targetV = 2447 + -51.2*diagonaldist + 0.753*diagonaldist*diagonaldist + -0.00437*diagonaldist*diagonaldist*diagonaldist + 0.0000091*diagonaldist*diagonaldist*diagonaldist*diagonaldist;
        }

//        if (gamepad1.dpad_left) flickys.setPosition(flickdown);
//        if (gamepad1.dpad_right) flickys.setPosition(flickup);
//





        if (gamepad1.rightStickButtonWasPressed()) {
            follower.followPath(pathChain);
        }
        if (gamepad1.rightStickButtonWasReleased()) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            slowMode = true;
            headingLock = false;
        }

        if (gamepad1.dpad_left) {
            follower.setPose(new Pose(72,9,(Math.toRadians(90))));
        }


        if (gamepad1.left_stick_button) {
            headingLock = false;
        }


//        if (gamepad1.right_trigger > 0.2 && !intakepressed) {
//            intakepressed = true;
//            if (intaketoggle) {
//                intaketoggle = false;
//            } else if (!intaketoggle) {
//                intaketoggle = true;
//            }
//        }
//        if (gamepad1.right_trigger < 0.2) {
//            intakepressed = false;
//        }


//        if (intaketoggle) {
//            intake.setPower(-1);
//        }
//        if (!intaketoggle) {
//            intake.setPower(0);
//        }

        if (gamepad1.left_bumper) {
            headingLock = true;
        }

        if (gamepad1.dpad_right) {
            slowMode = false;
        }


        if (gamepad1.dpad_down) {
            holdshooting = true;
        }
        if (gamepad1.dpad_up) {
            holdshooting = false;
        }


        turnerror = headinglockangle - Math.toDegrees(botHeading);
        controller.updateError(turnerror);

        if (headingLock)
            follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, controller.run(), true);
        else if (slowMode)
            follower.setTeleOpDrive(-gamepad2.left_stick_y*0.5, -gamepad2.left_stick_x*0.5, -gamepad2.right_stick_x*0.5, true);
        else
            follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x, true);


        if (gamepad1.y) {
        }
        if (gamepad1.b) {


            telemetry.addData("diag dist", diagonaldist);
            telemetry.addData("error", turnerror);
            telemetry.addData("targetV", targetV);
            telemetry.addData("headinglockangle", headinglockangle);
            telemetry.update();


        }
















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

        flywheel.setPower(kP * error + kV * targetV);

        telemetry.addData("targetV", targetV);
        telemetry.addData("targetV", far);
        telemetry.addData("actual velocity", flywheel.getVelocity());
        telemetry.update();



    }}


/// OPERATOR CONTROLS AS FOLLOWS:

/// left trigger: ------
/// left bumper: enable heading lock
/// right trigger: toggle intake off / outtake
/// right bumper: intake in
/// dpad up: enable defense brakes
/// dpad down: disable defense brakes
/// dpad left: reset pose
/// dpad right: slowmode off
/// a: reset spindexer plate
/// b: park, turn flywheel off if held
/// x: shake spindexer
/// y: retract park
/// left stick button: reset shooting cycle
/// right stick button: hold to auto park, release to manual drive


///ADD SLOWMODE TO BLUE