package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ifPractice extends OpMode {
    double motorSpeed= 0;
    @Override
    public void init() {
    }

    @Override
    public void loop() {

                if (gamepad1.bWasPressed()){
                    motorSpeed=motorSpeed+1;

                    telemetry.addData("Motor speed", motorSpeed);

                }
                else if (gamepad1.aWasPressed() && motorSpeed>0 ){
                    motorSpeed=motorSpeed-1;

                    telemetry.addData("Motor speed", motorSpeed);

                }
                else if (gamepad1.a && motorSpeed == 0){
                    telemetry.addData("Motor speed",motorSpeed);

                }
                else {
                    telemetry.addData("Motor Speed", motorSpeed);

        }
    }
}
