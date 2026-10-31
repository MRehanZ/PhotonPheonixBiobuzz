package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MotorPracticeTwo extends OpMode {
    TestBench bench = new TestBench();
    @Override
    public void loop() {
        bench.setMotorSpeed(0.5);
        telemetry.addData("Motor Rev", bench.getMotorRev());
    }

    @Override
    public void init() {
        bench.init(hardwareMap);
    }
}
