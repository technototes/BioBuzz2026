package org.firstinspires.ftc.teamcode.twenty403.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.technototes.library.command.Command;
import org.firstinspires.ftc.teamcode.twenty403.commands.driving.JoystickDriveCommand;

public class EZCmd {

    public static class Drive {

        public static Command AutoAim() {
            return Command.create(
                () -> JoystickDriveCommand.faceTagMode = !JoystickDriveCommand.faceTagMode
            );
        }

        public static Command ResetGyro(Follower follower) {
            return Command.create(() ->
                follower.setPose(new Pose(follower.pose().x(), follower.pose().y(), 0.0))
            );
        }
    }
}
