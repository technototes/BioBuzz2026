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
    // Ran on Oct 9 2026
    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        c.xPodName.set("fbodo");
        c.yPodName.set("strafeodo");
        c.imuName.set("imu");
        c.xPodOffset.set(1.73);
        c.yPodOffset.set(2.12);
        c.forwardTicksToInches.set(0.00177);
        c.strafeTicksToInches.set(0.00204);
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
        Controller primaryTranslationalForward = Controller.proportional(0.27971881633553536);
        Controller secondaryTranslationalForward = Controller.proportional(0.10334862841155307);
        Controller primaryTranslationalLateral = Controller.proportional(0.48144662310086206);
        Controller secondaryTranslationalLateral = Controller.proportional(0.17788166274507058);

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

        c.coast.set(Controller.proportionalFeedforward(0.013777419288337794));
        c.brake.set(Controller.proportionalFeedforward(0.011710806395087125));

        c.headingFeedback.set(Controller.proportional(14.24639824160929));
        c.headingBrakeCoefficients.set(
            Vector2D.cartesian(0.04756241922901685, 0.00551561844341756)
        );

        c.linearBrakeCoefficients.set(Matrix.diag(0.05735053908285078, 0.058135013547529826));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0022600197083254797, 0.0012621399803019945));

        c.maxAchievableForwardVelocity.set(71.88837428192382);
        c.maxAchievableStrafeVelocity.set(62.44050152401113);
        c.naturalForwardDeceleration.set(27.50243954601577);
        c.naturalStrafeDeceleration.set(59.156372270294554);
    });
}
