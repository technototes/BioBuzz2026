package org.firstinspires.ftc.teamcode.twenty403.validation;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.technototes.library.structure.ValidationOpMode;
import org.firstinspires.ftc.teamcode.twenty403.Setup;

@TeleOp(name = "Windmill Test", group = "validators")
public class TransferValidator extends ValidationOpMode {

    private CRServo transferMotor;
    public static double intakeVelocity = 0.75;

    @Override
    public void init() {
        super.init();
        transferMotor = this.hardwareMap.get(CRServo.class, Setup.HardwareNames.SPIN);
    }

    @Override
    public void loop() {
        super.loop();

        addLine("Press right bumper to spin intake");
        if (this.gamepad1.right_bumper) {
            transferMotor.setPower(intakeVelocity);
        }
    }
}
