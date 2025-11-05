package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;


import dev.nextftc.core.components.Component;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.Positionable;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;

public class Claw implements Subsystem {
    // put hardware, commands, etc here
    public static final Claw INSTANCE = new Claw();
    private Claw() {}
    private ServoEx servo = new ServoEx("servo");

    public Command open = new SetPositions(

            servo.to(1)).requires(this);

    public Command close = new SetPositions(

            servo.to(0)).requires(this);


    //private final SetPosition open = new SetPosition(servo, 0);
    //private final SetPosition close = new SetPosition(servo, 1);
}
