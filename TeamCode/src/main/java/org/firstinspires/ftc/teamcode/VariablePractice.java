package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class VariablePractice extends OpMode {
    @Override
    public void init() {
        int teamNumber =32086;
        telemetry.addData("Team Number", teamNumber);
    }

    @Override
    public void loop() {
        boolean ClawClosed = true;
       telemetry.addData("hi",ClawClosed) ;
    }
}