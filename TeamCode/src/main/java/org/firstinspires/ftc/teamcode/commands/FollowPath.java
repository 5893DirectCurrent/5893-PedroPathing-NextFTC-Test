package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

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

public class FollowPath extends Command {

//        private final Set<Subsystem> subsystems;
//        private final boolean interruptible = true;
//
        public FollowPath(PathChain path) {
             follower.followPath(path);
        }

        @Override
        public boolean isDone() {
            return false; // Whether or not the command is done
        }

        @Override
        public void start() {
            // Executed when the command begins

        }

        @Override
        public void update() {
            // Executed on every update of the command
        }

        @Override
        public void stop(boolean interrupted) {
            // Executed when the command ends
        }
    }

