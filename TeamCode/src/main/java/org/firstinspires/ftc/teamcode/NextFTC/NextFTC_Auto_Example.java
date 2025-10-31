package org.firstinspires.ftc.teamcode.NextFTC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.systems.Claw;

@Autonomous(name = "NextFTC Autonomous Program Java")
public class NextFTC_Auto_Example extends NextFTCOpMode {
    public NextFTC_Auto_Example() {
        super(Claw.INSTANCE);
    }

    public Command firstRoutine() {
        return new SequentialGroup(
                Claw.INSTANCE.close(),
                new ParallelGroup(

                        Claw.INSTANCE.open()
                ),
                new Delay(2),
                new ParallelGroup(
                        Claw.INSTANCE.close()

                )
        );
    }

    @Override
    public void onStartButtonPressed() {
        firstRoutine().invoke();
    }
}

