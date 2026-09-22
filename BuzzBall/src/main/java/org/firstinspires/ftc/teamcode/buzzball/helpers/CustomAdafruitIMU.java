package org.firstinspires.ftc.teamcode.buzzball.helpers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.revhub.localizers.CustomIMU;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technototes.library.hardware.sensor.AdafruitIMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class CustomAdafruitIMU implements CustomIMU {

    public static double initialZeroDegrees = 0;
    public double curZero;
    AdafruitIMU imu;

    public void initialize(HardwareMap hardwareMap, String hardwareMapName) {
        AdafruitBNO055IMU _imu = hardwareMap.get(AdafruitBNO055IMU.class, hardwareMapName);
        imu = new AdafruitIMU(_imu, hardwareMapName, AdafruitIMU.Orientation.Pitch);
        curZero = Math.toRadians(initialZeroDegrees);
    }

    @Override
    public double getHeading() {
        return imu.getHeading(AngleUnit.RADIANS) - curZero;
    }

    @Override
    public void resetYaw() {
        curZero = imu.getHeading(AngleUnit.RADIANS);
    }
}
