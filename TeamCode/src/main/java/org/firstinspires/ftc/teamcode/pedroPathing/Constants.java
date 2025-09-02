package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;


public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(4.99)
            .forwardZeroPowerAcceleration(-51.52186169612585)
            .lateralZeroPowerAcceleration(-48.482894898440705)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.2, 0.2, 0.1, 0.1))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0.01, 0.01, 0, 0.1));


    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(67.14043228269563)
            .yVelocity(61.276640163631896);


    public static PinpointConstants localizerConstants = new PinpointConstants()
    .forwardPodY(-6)
    .strafePodX(-7)
    .distanceUnit(DistanceUnit.INCH)
    .hardwareMapName("pinpoint")
    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.007,
            500,
            1.5,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}