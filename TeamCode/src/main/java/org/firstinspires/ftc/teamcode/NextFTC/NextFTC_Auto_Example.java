package org.firstinspires.ftc.teamcode.NextFTC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

import dev.nextftc.core.components.SubsystemComponent;

@Autonomous(name = "NextFTC Autonomous Program Java")
public class NextFTC_Auto_Example extends NextFTCOpMode {
    public NextFTC_Auto_Example() {
        addComponents(
                new SubsystemComponent(Claw.INSTANCE)
                );
    }

    public Command firstRoutine() {
        return new SequentialGroup(
                Claw.INSTANCE.close,
                new ParallelGroup(

                        Claw.INSTANCE.open
                ),
                new Delay(2),
                new ParallelGroup(
                        Claw.INSTANCE.close

                )
        );
    }

    @Override
    public void onStartButtonPressed() {
        firstRoutine().invoke();
    }
}

