package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
         .mass(5)
         .forwardZeroPowerAcceleration(-52.86)
         .lateralZeroPowerAcceleration(-70.12)
         .useSecondaryTranslationalPIDF(true)
         .useSecondaryHeadingPIDF(true)
         .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.03,
                    0,
                    0,
                    0.015
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.4,
                    0,
                    0.005,
                    0.0006
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0,
                    0.01
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5,
                    0,
                    0.1,
                    0.0005
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1,
                    0,
                    0.00035,
                    0.6,
                    0.015
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.000005,
                    0.6,
                    0.01
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6)
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(65.53)
            .yVelocity(52.39);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

