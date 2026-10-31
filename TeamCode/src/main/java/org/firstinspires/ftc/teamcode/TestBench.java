package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBench {
    private DcMotor motor; //motor
    private double ticksPerRev;

    public void init(HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class, "motor1");

       // motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRev = motor.getMotorType().getTicksPerRev();
    }
    public void setMotorSpeed(double speed){
        //accepts value from -1.0 to 1.0
        motor.setPower(speed);
    }
    public double getMotorSpeed() {
        return motor.getPower();
    }
    public double getMotorRev(){
        //return motor.getCurrentPosition()/2786;

       // return ticksPerRev; //2786
        return motor.getCurrentPosition();
    }
}
