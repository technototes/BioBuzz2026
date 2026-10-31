package org.firstinspires.ftc.teamcode.twenty403.opmodes.autos;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.command.SequentialCommandGroup;
import com.technototes.library.command.WaitCommand;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.library.util.Alliance;
import org.firstinspires.ftc.teamcode.twenty403.Hardware;
import org.firstinspires.ftc.teamcode.twenty403.PedroConstants;
import org.firstinspires.ftc.teamcode.twenty403.Robot;
import org.firstinspires.ftc.teamcode.twenty403.commands.auto.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.twenty403.controls.DriverController;
import org.firstinspires.ftc.teamcode.twenty403.helpers.StartingPosition;

@Autonomous(name = "Park", preselectTeleOp = "Two Controller Drive \uD83D\uDDFF")
@SuppressWarnings("unused")
public class SouthJustPark extends CommandOpMode {

    public Robot robot;
    public DriverController controls;
    public Hardware hardware;
    private PanelsTelemetry panelsTelemetry;
    private Limelight3A limelight;

    @Override
    public void uponInit() {
        hardware = new Hardware(hardwareMap);
        robot = new Robot(hardware, Alliance.BLUE, StartingPosition.Net);
        panelsTelemetry = PanelsTelemetry.INSTANCE;
        robot.follower.setPose(PedroConstants.Poses.South);
        CommandScheduler.scheduleForState(
            new SequentialCommandGroup(
                new DriveAutoCommand(robot.follower, 0.5, 0),
                new WaitCommand(.3),
                new DriveAutoCommand(robot.follower, 0, 0)
            ),
            OpModeState.RUN
        );
    }

    @Override
    public void runLoop() {}
}
