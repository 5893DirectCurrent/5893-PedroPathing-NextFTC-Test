package org.firstinspires.ftc.teamcode.systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.teamcode.TestAuto;

public class RobotSubsystems {

    public CRServo Servo = null;

    public LinearOpMode myOpMode;


    public RobotSubsystems(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public RobotSubsystems(TestAuto testAuto) {
    }

    public void initialize(boolean showTelemetry) {

        Servo = hardwareMap.get(CRServo.class, "servo");

    }

//    public void Spin(double power, int time){
//        final ElapsedTime runtime = new ElapsedTime();
//        runtime.milliseconds();
//        runtime.reset();
//        while (myOpMode.opModeIsActive()) {
//            Servo.setPower(power);
//            if(runtime.time() >= time){
//                break;
//            }
//        }
//    }
}
