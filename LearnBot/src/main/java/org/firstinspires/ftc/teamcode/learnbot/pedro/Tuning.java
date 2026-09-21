package org.firstinspires.ftc.teamcode.learnbot.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OctoQuadLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.learnbot.components.PedroDrivebase;
import org.firstinspires.ftc.teamcode.learnbot.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.learnbot.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.learnbot.pedro.procedures.OctoQuadTuner;
import org.firstinspires.ftc.teamcode.learnbot.pedro.procedures.Tests;

public class Tuning {

    // Tuners go here
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure octoquadTuner() {
        return new OctoQuadTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
            hardwareMap ->
                new OctoQuadLocalizer(hardwareMap, PedroDrivebase.Config.localizerConfig),
            hardwareMap -> new Mecanum(hardwareMap, PedroDrivebase.Config.drivetrainConfig)
        );
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
            hardwareMap -> new Mecanum(hardwareMap, PedroDrivebase.Config.drivetrainConfig),
            hardwareMap ->
                new OctoQuadLocalizer(hardwareMap, PedroDrivebase.Config.localizerConfig),
            () -> new Foresight(PedroDrivebase.Config.foresightConfig)
        );
    }
}
