package org.firstinspires.ftc.learnbot.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.logger.Loggable;
import com.technototes.library.structure.CommandOpMode;
import org.firstinspires.ftc.learnbot.Hardware;
import org.firstinspires.ftc.learnbot.Setup;
import org.firstinspires.ftc.learnbot.components.Slides;

@Configurable
@SuppressWarnings("unused")
@TeleOp(name = "Slides Testing")
public class SlidesTele extends CommandOpMode implements Loggable {

    public Hardware hardware;
    public Slides.Component slides;

    @Override
    public void uponInit() {
        hardware = new Hardware(hardwareMap);
        slides = new Slides.Component(hardware.slides);

        driverGamepad.ps_triangle.whenPressed(Slides.Commands.UP());
        driverGamepad.ps_circle.whenPressed(Slides.Commands.MIDDLE());
        driverGamepad.ps_square.whenPressed(Slides.Commands.LOW());
        driverGamepad.ps_cross.whenPressed(Slides.Commands.ZERO());

    }
}