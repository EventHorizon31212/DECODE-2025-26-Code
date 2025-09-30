package org.firstinspires.ftc.teamcode.pedroPathing.auto;
import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "autoTest", group = "Robot")
public class autoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private final Pose startPose = new Pose(38, 12, Math.toRadians(90));
    private final Pose startPose2 = new Pose(120, 120, Math.toRadians(35));
    private final Pose cPoint = new Pose(115, 50);
    private final Pose endPose = new Pose(120, 120, Math.toRadians(35));
    private final Pose endPose2 = new Pose(42,83.5,Math.toRadians(180));
    private PathChain testPath, testPath2;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;
    AprilTagDetection myAprilTagDetection;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    public void buildPaths() {
        testPath = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, cPoint, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
        testPath2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose2, endPose2))
                .setLinearHeadingInterpolation(startPose2.getHeading(), endPose2.getHeading())
                .build();
    }//end path building

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }//end setPathState init

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(testPath, true);
                    setPathState(1);
                    break;
                }
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(testPath2, true);
                    setPathState(2);
                    break;
                }
        }//end switch
    }//end autonomousPathUpdate

    @Override
    public void loop() {
        //actually running the code
        follower.update();
        autonomousPathUpdate();
        //pose update
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }//end loop

    @Override
    public void init() {
        //path timer and build paths init
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        //april tag init
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
        //vision portal init
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        telemetry.addData("Status: ", "Initialized");
    }//end init

    @Override
    public void start() {
        setPathState(0);
    } //end start
} //end class

