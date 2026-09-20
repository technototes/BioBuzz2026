package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OctoQuadConfig;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static Follower create(HardwareMap h) {
        return new Follower(new OctoQuadLocalizer(), Localizer, Foresight);
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("rl");
        c.backRightName.set("rr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
        c.name.set("octoquad");
        c.xPodPort.set(7);
        c.yPodPort.set(6);
        c.ticksPerUnit.set(505.316944406);
        c.xPodOffset.set(-2.34251968503937);
        c.yPodOffset.set(0.688976377952756);
        c.xPodDirection.set(OctoQuad.EncoderDirection.FORWARD);
        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
        c.headingScalar.set(1.0144895113606114);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.17242498230829773);
        Controller secondaryTranslationalForward = Controller.proportional(0.06370642368253522);
        Controller primaryTranslationalLateral = Controller.proportional(0.22409480643857885);
        Controller secondaryTranslationalLateral = Controller.proportional(0.08279704305554568);

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

        c.coast.set(Controller.proportionalFeedforward(0.017881248203488014));
        c.brake.set(Controller.proportionalFeedforward(0.015199060972964811));

        c.headingFeedback.set(Controller.proportional(2.415440556391569));
        c.headingBrakeCoefficients.set(
            Vector2D.cartesian(0.04427148791685309, 0.0024816585608359036)
        );

        c.linearBrakeCoefficients.set(Matrix.diag(0.061406279185640913, 0.04023543050788815));
        c.quadraticBrakeCoefficients.set(Matrix.diag(9.568312745336765E-4, 0.0012968587256159201));

        c.maxAchievableForwardVelocity.set(57.977352408951944);
        c.maxAchievableStrafeVelocity.set(48.71239910399553);
        c.naturalForwardDeceleration.set(31.96456257414348);
        c.naturalStrafeDeceleration.set(53.50127263937453);
    });
}
