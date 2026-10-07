package org.firstinspires.ftc.teamcode.twenty403;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PedroConstants {

    public static Follower create(HardwareMap h) {
        return new Follower(
            new TwoWheelLocalizer(h, localizerConfig),
            new Mecanum(h, drivetrainConfig),
            new Foresight(foresightConfig)
        );
    }

    // TODO: Run the tuners on the bot, fill these it with the results:
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("rl");
        c.backRightName.set("rr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        c.xPodName.set("fbodo");
        c.yPodName.set("strafeodo");
        c.imuName.set("imu");
        c.xPodOffset.set(-0.044544361213595896);
        c.yPodOffset.set(-6.755979305146669);
        c.forwardTicksToInches.set(0.0019973615715189914);
        c.strafeTicksToInches.set(0.002071152863611527);
        c.xPodDirection.set(Encoder.FORWARD);
        c.yPodDirection.set(Encoder.FORWARD);
        c.imu.set(
            new RevHubIMU(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
            )
        );
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.26316799030345517);
        Controller secondaryTranslationalForward = Controller.proportional(0.09723354043891597);
        Controller primaryTranslationalLateral = Controller.proportional(0.4333549343156143);
        Controller secondaryTranslationalLateral = Controller.proportional(0.16011306877251263);

        c.forwardTranslational.set(
            Controller.piecewise(secondaryTranslationalForward).put(
                2.5,
                primaryTranslationalForward
            )
        );
        c.strafeTranslational.set(
            Controller.piecewise(secondaryTranslationalLateral).put(
                2.5,
                primaryTranslationalLateral
            )
        );

        c.coast.set(Controller.proportionalFeedforward(0.013402344630944559));
        c.brake.set(Controller.proportionalFeedforward(0.011391992936302874));

        c.headingFeedback.set(Controller.proportional(6.262921484299736));
        c.headingBrakeCoefficients.set(
            Vector2D.cartesian(0.059888673975135163, 0.0037857270380500523)
        );

        c.linearBrakeCoefficients.set(Matrix.diag(0.09333249940119288, 0.0595602507219176));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012810241980275724, 0.0016948956191025326));

        c.maxAchievableForwardVelocity.set(73.25426377980192);
        c.maxAchievableStrafeVelocity.set(41.95581906838906);
        c.naturalForwardDeceleration.set(34.01811460082023);
        c.naturalStrafeDeceleration.set(83.19033430043984);
    });
}
