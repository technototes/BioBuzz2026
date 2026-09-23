package org.firstinspires.ftc.teamcode.twenty403.validation;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.technototes.library.structure.ValidationOpMode;
import org.firstinspires.ftc.teamcode.twenty403.Setup;

public class TransferValidator extends ValidationOpMode {

    private DcMotorEx transferMotor;
    public static double intakeVelocity = 2;

    @Override
    public void init() {
        super.init();
        transferMotor = this.hardwareMap.get(DcMotorEx.class, Setup.HardwareNames.TRANSFERMOTOR);
    }

    @Override
    public void loop() {
        super.loop();

        addLine("Press right bumper to spin intake");
        if (this.gamepad1.right_bumper) {
            transferMotor.setVelocity(intakeVelocity);
        }
    }
}
