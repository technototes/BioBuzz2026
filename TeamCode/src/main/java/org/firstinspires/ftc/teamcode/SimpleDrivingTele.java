package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@SuppressWarnings("unused")
@TeleOp(name = "driving")
public class SimpleDrivingTele extends LinearOpMode {

    public DcMotorEx fl, fr, rl, rr;

    @Override
    public void runOpMode() throws InterruptedException {
        // First, get the hardware
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
        // Motors on opposite sides of the bot need to spin opposite directions
        fl.setDirection(Direction.REVERSE);
        rl.setDirection(Direction.REVERSE);
        fr.setDirection(Direction.FORWARD);
        rr.setDirection(Direction.FORWARD);
        waitForStart();
        while (opModeIsActive()) {
            double fb = this.gamepad1.left_stick_y;
            fl.setPower(-fb);
            rl.setPower(-fb);
            fr.setPower(-fb);
            rr.setPower(-fb);
            telemetry.addData("info", 1);
            telemetry.update();
        }
    }
}
