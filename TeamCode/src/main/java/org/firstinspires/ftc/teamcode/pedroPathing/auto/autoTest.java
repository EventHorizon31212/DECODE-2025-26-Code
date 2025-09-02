package org.firstinspires.ftc.teamcode.pedroPathing.auto;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "autoTest", group = "Robot")
public class autoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private final Pose startPose = new Pose(70, 70, Math.toRadians(90));
    private final Pose cPoint = new Pose(115, 50);
    private final Pose cPoint2 = new Pose(135,3);
    private final Pose endPose = new Pose(120, 120, Math.toRadians(270));
    private PathChain testPath, testPath2;

    public void buildPaths() {
        testPath = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, cPoint, cPoint2, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
            follower.followPath(testPath, true);
            setPathState(1);
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        setPathState(0);
    }
}

