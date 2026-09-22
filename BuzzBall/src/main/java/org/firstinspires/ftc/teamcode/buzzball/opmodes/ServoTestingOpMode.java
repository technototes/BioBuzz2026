package org.firstinspires.ftc.teamcode.buzzball.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.technototes.library.structure.CommandOpMode;
import com.technototes.library.util.Alliance;
import org.firstinspires.ftc.teamcode.buzzball.Hardware;
import org.firstinspires.ftc.teamcode.buzzball.Robot;
import org.firstinspires.ftc.teamcode.buzzball.Setup;
import org.firstinspires.ftc.teamcode.buzzball.controls.TestingController;
import org.firstinspires.ftc.teamcode.buzzball.helpers.StartingPosition;

@TeleOp(name = "TestingController")
public class ServoTestingOpMode extends CommandOpMode {

    public Robot robot;
    public Setup setup;
    public TestingController controls;
    public Hardware hardware;

    @Override
    public void uponInit() {
        hardware = new Hardware(hardwareMap);
        robot = new Robot(hardware, Alliance.NONE, StartingPosition.Unspecified);
        controls = new TestingController(codriverGamepad, robot);
        //        CommandScheduler
        //
        //            .scheduleForState(new DroneStart(robot.drone), OpModeState.INIT);
    }
}
