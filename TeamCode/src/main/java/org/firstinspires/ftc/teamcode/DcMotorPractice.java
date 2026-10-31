package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DcMotorPractice extends OpMode {
    TestBench bench = new TestBench();
    public void init(){
       bench.init(hardwareMap);
        bench.setMotorSpeed(0);

    }
    public void loop(){
        if (gamepad1.aWasPressed() && bench.getMotorSpeed()>=0){
            bench.setMotorSpeed(bench.getMotorSpeed() +0.1);
        }
        else if (gamepad1.bWasPressed() && bench.getMotorSpeed()>0){
            bench.setMotorSpeed(bench.getMotorSpeed()-0.1);

        }
        else if (bench.getMotorSpeed()<0){
            bench.setMotorSpeed(0);
        }
        // bench.setMotorSpeed(0.5);
    telemetry.addData("motor speed",bench.getMotorSpeed());
    }
}
