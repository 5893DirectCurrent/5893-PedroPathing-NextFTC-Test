package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;



import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.systems.Claw;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;




@Autonomous(name = "Test Auto", group = "Examples")
public class NextFTC_Auto extends NextFTCOpMode {
    public NextFTC_Auto() {
        super(Claw.INSTANCE);
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose firstPose = new Pose(48, 0, Math.toRadians(0));
    private final Pose secondPose = new Pose(48, -72, Math.toRadians(180));
    private final Pose finalPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose curvePose = new Pose(15, -60, Math.toRadians(90));


 //   private Path firstMove;
    private PathChain firstMove, secondMove, finalMove;


//    private RobotSubsystems robot = new RobotSubsystems(this);





    public void buildPaths() {
        /* This is our firstMove path. We are using a BezierLine, which is a straight line. */
        firstMove = follower.pathBuilder()
                .addPath(new BezierLine(startPose, firstPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstPose.getHeading())
                .build();


        secondMove = follower.pathBuilder()
                .addPath(new BezierLine(firstPose, secondPose))
                .setLinearHeadingInterpolation(firstPose.getHeading(), secondPose.getHeading())
                .build();


        finalMove = follower.pathBuilder()
                .addPath(new BezierCurve(secondPose, curvePose, finalPose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), finalPose.getHeading())
                .build();

    }

    public Command firstRoutine() {
        return new SequentialGroup(
                new FollowPath(firstMove),
                Claw.INSTANCE.close(),
                new FollowPath(secondMove),
                new ParallelGroup(
                        new FollowPath(finalMove),
                        Claw.INSTANCE.close()
                )
        );
    }



    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
////        autonomousPathUpdate();
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//
//    /**
//     * This method is called once at the init of the OpMode.
//     **/
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//
//
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//
//    }

    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        firstRoutine().invoke();
    }

}

