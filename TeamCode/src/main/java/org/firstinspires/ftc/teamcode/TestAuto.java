package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.widget.Spinner;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.systems.RobotSubsystems;

@Autonomous(name = "Test Auto", group = "Examples")
public class TestAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose firstPose = new Pose(48, 0, Math.toRadians(0));
    private final Pose secondPose = new Pose(48, -72, Math.toRadians(180));
    private final Pose finalPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose curvePose = new Pose(15, -60, Math.toRadians(90));

    private CRServo Servo;


    private Path firstMove;
    private PathChain secondMove, finalMove;




//    private RobotSubsystems robot = new RobotSubsystems(this);

    public void Spin(double power, int time){
        final ElapsedTime runtime = new ElapsedTime();
        runtime.milliseconds();
        runtime.reset();
        while (runtime.time() < time) {
            Servo.setPower(power);
        }
        Servo.setPower(0);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Servo = hardwareMap.get(CRServo.class, "servo");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    public void buildPaths() {
        /* This is our firstMove path. We are using a BezierLine, which is a straight line. */
        firstMove = new Path(new BezierLine(startPose, firstPose));
        firstMove.setLinearHeadingInterpolation(startPose.getHeading(), firstPose.getHeading());

    /* Here is an example for Constant Interpolation
    firstMove.setConstantInterpolation(startPose.getHeading()); */

        /* This is our firstMove PathChain. We are using a single path with a BezierLine, which is a straight line. */
        secondMove = follower.pathBuilder()
                .addPath(new BezierLine(firstPose, secondPose))
                .setLinearHeadingInterpolation(firstPose.getHeading(), secondPose.getHeading())
                .build();

        /* This is our secondMove PathChain. We are using a single path with a BezierLine, which is a straight line. */
        finalMove = follower.pathBuilder()
                .addPath(new BezierCurve(secondPose, curvePose, finalPose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), finalPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Spin(1,2000);
                follower.followPath(firstMove);
                setPathState(2);
                break;
//            case 1:
//
//                setPathState(2);
//                break;
            case 2:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the firstPose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(secondMove, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the secondPose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(finalMove, true);
                    setPathState(-1);
                }
                break;

        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/


    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}