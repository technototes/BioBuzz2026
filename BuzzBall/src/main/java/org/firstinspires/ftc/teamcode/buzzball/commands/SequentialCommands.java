package org.firstinspires.ftc.teamcode.buzzball.commands;

import com.technototes.library.command.Command;
import com.technototes.library.command.ParallelCommandGroup;
import com.technototes.library.command.SequentialCommandGroup;
import com.technototes.library.command.WaitCommand;
import org.firstinspires.ftc.teamcode.buzzball.Robot;

public class SequentialCommands {

    static TeleCommands t = new TeleCommands();

    public static Command HighBasketScore(Robot r) {
        return new SequentialCommandGroup(
            t.DepositClawClose(r),
            new WaitCommand(0.25),
            new ParallelCommandGroup(
                t.SetHighBasket(r),
                t.SlidesHigh(r),
                t.ArmCompensateOn(r),
                t.WristHoriz(r)
            )
        );
    }

    public static Command LowBasketScore(Robot r) {
        return new SequentialCommandGroup(
            t.DepositClawClose(r),
            new WaitCommand(0.25),
            new ParallelCommandGroup(
                t.SetLowBasket(r),
                t.SlidesLow(r),
                t.ArmCompensateOn(r),
                t.WristHoriz(r)
            )
        );
    }

    public static Command PlaceAndRetract(Robot r) {
        return new SequentialCommandGroup(
            t.DepositClawOpen(r),
            new WaitCommand(0.5),
            new ParallelCommandGroup(t.ArmCompensateOff(r), t.SlidesZero(r), t.ArmDown(r))
        );
    }
}
