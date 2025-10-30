package org.firstinspires.ftc.teamcode.systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotSubsystems {

    private CRServo Servo = null;

    private OpMode myOpMode;


    public RobotSubsystems(OpMode opmode) {
        myOpMode = opmode;
    }

//    public RobotSubsystems(TestAuto testAuto) {
//    }

    public void initialize() {

        Servo = hardwareMap.get(CRServo.class, "servo");

    }

    public void Spin(double power, int time){
        final ElapsedTime runtime = new ElapsedTime();
        runtime.milliseconds();
        runtime.reset();
        while (true) {
            Servo.setPower(power);
            if (runtime.time()>=time){
                Servo.setPower(0);
                break;
            }
        }
    }
}
