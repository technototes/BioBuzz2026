package org.firstinspires.ftc.sixteen750;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.sixteen750.Setup.HardwareNames;

@Configurable
public class AutoConstants {

    // note these need to be measured:
    public static double botWeightKg = 10.1;
    public static double robotLength = 17.5;
    public static double robotWidth = 11.5;

    // These come from Tuners:
    public static double xvelocity = 84.5;
    public static double yvelocity = 74.75;
    public static double fwdDeceleration = -42.8;
    public static double latDeceleration = -71.1;
    // Predictive Braking doesn't need centripetal scaling
    public static double centripetalScaling = 0.0005;

    // Predictive Braking tuning (Measured on the bot on June 17)
    public static double kQuadratic = 8.6266e-4;
    public static double kLinear = 0.1149;
    public static double kP = 0.1;

    // These are hand tuned to work how we want
    public static double brakingStrength = 1;
    public static double brakingStart = 1;

    // The percent of a path that must be complete for Pedro to decide it's done
    // For predictive braking, this is supposed to be lower, so I dropped it from
    // 0.99 to 0.95
    public static double TValueConstraint = 0.95;
    // Time, in *milliseconds*, to let the follower algorithm correct
    // before the path is considered "complete".
    public static double timeoutConstraint = 60;
    // The maximum velocity (in inches/second) the bot can be moving while still
    // saying the path is complete.
    public static double acceptableVelocity = 2;
    // The maximum RawDistance (in inches) the bot can be from the path end
    // while still saying the path is complete.
    public static double acceptableDistance = 2.0;
    // The maximum heading error (in degrees) the bot can be from the path end
    // while still saying the path is complete.
    public static double acceptableHeading = 1.5;

    @Configurable
    public static class DriveEncoderConfig {

        public static double fwdTicksToInches = 0.008;
        public static double strafeTicksToInches = -0.009;
        public static double turnTicksToInches = 0.018;
    }

    @Configurable
    public static class OTOSConfig {

        public static SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(1.50, 0.0, 180);
        public static double linearscalar = -1.08; //1.9
        public static double angularscalar = 0.9;
    }

    @Configurable
    public static class TwoWheelConfig {

        public static String forwardName = HardwareNames.ODOFB;
        public static String strafeName = HardwareNames.ODORL;
        public static double forwardTicksToInches = ((17.5 / 25.4) * 2 * Math.PI) / 8192; // 5.42, 5.47, 5.49
        public static double strafeTicksToInches = ((17.5 / 25.4) * 2 * Math.PI) / 8192; // 5.37, 5.39, 5.38
        public static double forwardPodYOffset = -3.9; // From Colin's CAD 10/31
        public static double strafePodXOffset = -4.124; // From Colin's CAD 10/31
        public static boolean forwardReversed = true;
        public static boolean strafeReversed = false;
        public static RevHubOrientationOnRobot.LogoFacingDirection logoDir =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public static RevHubOrientationOnRobot.UsbFacingDirection usbDir =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
    }

    /*
    public static DriveEncoderConstants getEncoderConstants() {
        return new DriveEncoderConstants()
                .forwardTicksToInches(DriveEncoderConfig.fwdTicksToInches)
                .strafeTicksToInches(DriveEncoderConfig.strafeTicksToInches)
                .turnTicksToInches(DriveEncoderConfig.turnTicksToInches)
                .robotLength(robotLength)
                .robotWidth(robotWidth)
                .rightFrontMotorName(HardwareNames.FR_DRIVE_MOTOR)
                .rightRearMotorName(HardwareNames.RR_DRIVE_MOTOR)
                .leftRearMotorName(HardwareNames.RL_DRIVE_MOTOR)
                .leftFrontMotorName(HardwareNames.FL_DRIVE_MOTOR)
                .leftFrontEncoderDirection(Encoder.FORWARD)
                .leftRearEncoderDirection(Encoder.REVERSE)
                .rightFrontEncoderDirection(Encoder.REVERSE)
                .rightRearEncoderDirection(Encoder.FORWARD);
    }

    public static OTOSConstants getOTOSConstants() {
        return new OTOSConstants()
            .hardwareMapName(HardwareNames.OTOS)
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(OTOSConfig.linearscalar)
            .angularScalar(OTOSConfig.angularscalar);
    }
    */

    public static Follower createFollower(HardwareMap hardwareMap) {
        return null;
    }
}
