package org.firstinspires.ftc.teamcode.learnbot.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

@SuppressWarnings("unused")
@TeleOp(name = "SimpleDriving")
public class SimpleDrivingTele extends LinearOpMode {

    public DcMotorEx fl, fr, rl, rr;
    public IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // First, get the hardware
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
            new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
            )
        );
        imu.resetYaw();
        // Motors on opposite sides of the bot need to spin opposite directions
        fl.setDirection(Direction.REVERSE);
        rl.setDirection(Direction.REVERSE);
        fr.setDirection(Direction.FORWARD);
        rr.setDirection(Direction.FORWARD);
        waitForStart();
        int mode = 0;
        while (opModeIsActive()) {
            double fwdBack = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if (gamepad1.leftBumperWasPressed() || gamepad1.rightBumperWasPressed()) {
                mode = (mode + 1) % 3;
            }
            if (gamepad1.leftTriggerWasPressed() || gamepad1.rightTriggerWasPressed()) {
                imu.resetYaw();
            }
            if (mode != 0) {
                double heading = mode == 1 ? 0 : botHeading;
                // Bot-relative driving
                double rotX = strafe * Math.cos(-heading) - fwdBack * Math.sin(-heading);
                double rotY = strafe * Math.sin(-heading) + fwdBack * Math.cos(-heading);
                // Scale to prevent clipping
                double denominator = Math.max(
                    Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate),
                    1
                );
                fl.setPower((rotY + rotX + rotate) / denominator);
                rl.setPower((rotY - rotX + rotate) / denominator);
                fr.setPower((rotY - rotX - rotate) / denominator);
                rr.setPower((rotY + rotX - rotate) / denominator);
            } else {
                double fbMagnitude = Math.abs(fwdBack);
                double sMagnitude = Math.abs(strafe);
                double rMagnitude = Math.abs(rotate);
                if (fbMagnitude >= sMagnitude && fbMagnitude >= rMagnitude) {
                    // We're driving forward/backward: Move all wheels in the same direction
                    fl.setPower(fwdBack);
                    fr.setPower(fwdBack);
                    rl.setPower(fwdBack);
                    rr.setPower(fwdBack);
                } else if (sMagnitude >= fbMagnitude && sMagnitude >= rMagnitude) {
                    // We're strafing left/right: Move opposite corners in the same direction
                    fl.setPower(strafe);
                    rr.setPower(strafe);
                    fr.setPower(-strafe);
                    rl.setPower(-strafe);
                } else {
                    // We're rotating:: Opposite sides move in opposite directions
                    fl.setPower(rotate);
                    rl.setPower(rotate);
                    fr.setPower(-rotate);
                    rr.setPower(-rotate);
                }
            }
            telemetry.addLine("Bumpers to change mode");
            telemetry.addLine("Triggers to reset angle");
            telemetry.addData("Angle", Math.toDegrees(botHeading));
            telemetry.addData("Control", mode == 0 ? "Dumb" : "Trig");
            telemetry.addData("Relative", mode == 2 ? "Field" : "Bot");
            telemetry.addData("fwd/back", fwdBack);
            telemetry.addData("strafe", strafe);
            telemetry.addData("rotate", rotate);
            telemetry.update();
        }
    }
}
