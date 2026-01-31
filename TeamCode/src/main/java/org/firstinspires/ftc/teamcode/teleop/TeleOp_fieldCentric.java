package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Field Centric TeleOp", group = "Robot")
//
public class TeleOp_fieldCentric extends OpMode {
    private final ElapsedTime runTime = new ElapsedTime();
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor intake;
    DcMotor shooter;
    Servo pusher;
    IMU imu;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        pusher = hardwareMap.get(Servo.class, "pusher");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to reset Yaw");
        if (gamepad1.y) {
            imu.resetYaw();
        }

        if (gamepad1.right_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1;
        double maxSpeed = 1;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        if (gamepad1.a) {
            intake.setPower(maxSpeed);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.b) {
            shooter.setPower(1);
        } else {
            shooter.setPower(0);
        }

        if (gamepad1.x) {
            pusher.setPosition(0.35);
        } else {
            pusher.setPosition(0.9);
        }

        if (gamepad1.left_bumper) {
            maxSpeed = 0.5;
        } else {
            maxSpeed = 1;
        }

        frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        backRight.setPower(maxSpeed * (backRightPower / maxPower));

        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.addData("Front Motors: ", "%4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back Motors:", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.addData("IMU Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }
}
