/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@Config("LauncherTuning")
@TeleOp(name = "Mec BioBuzz StarterBot Teleop", group = "StarterBot")
public class StarterBotTeleOp extends OpMode {
    private Limelight3A limelight;

    private final MecanumDrive drive = new MecanumDrive();
    double targetRPM;

    double error = 0;
    double goalX = -1.5;
    double forward, strafe, rotate;
    double kp = 0.035; // TODO: this might be too high 24 nov
    double kd = 0.002; // TODO: tune this 24 nov
    double lastError = 0;
    double curTime = 0;
    double lastTime = 0;
    final double angleTolerance = 0.18; // TODO: update this degree tolerance 24 nov

    // === Limelight Mounting Constants (TUNE THESE) ===
    final double CAMERA_HEIGHT_IN = 12.5;
    final double TARGET_HEIGHT_IN = 50; // TODO: this is your april tag, not your hive
    final double CAMERA_MOUNT_ANGLE = 60;

    // Launcher velocity PIDF gains. Must be non-zero or the launcher won't spin.
    public static double KP = 20;
    public static double KI = 0.0;
    public static double KD = 0.1;
    public static double KF = 12.5;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    org.firstinspires.ftc.robotcore.external.Telemetry dashboardTelemetry = dashboard.getTelemetry();

    // Declare OpMode members.

    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    //private CRServo leftIntakeServo = null;
    //private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * Launcher velocities in encoder ticks per second (28 ticks/rev).
     * The windmill (transfer) only runs once the launcher is above MIN.
     */

    double intakePower;

    boolean flywheelActived = false;


    @Override
    public void init() {

        drive.init(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "transfer");
        //leftIntakeServo = hardwareMap.get(CRServo.class, "left_flower");
        //rightIntakeServo = hardwareMap.get(CRServo.class, "right_flower");

        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(KP, KI, KD, KF));

        //leftIntakeServo.setPower(0);
        //rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        //rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        if (gamepad1.bWasPressed()) {
            limelight.pipelineSwitch(8); // AprilTag pipeline
            telemetry.addLine("Pipeline 8 - Red Goal");
        } else if (gamepad1.xWasPressed()) {
            limelight.pipelineSwitch(9);
            telemetry.addLine("Pipeline 9 - Blue Goal");
        }
    }

    @Override
    public void start() {
        limelight.start();
        resetRuntime();
        lastTime = getRuntime();
    }

    @Override
    public void loop() {
        // sense driving controls for mecanum
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;


        // TODO: Decide on a way to contorl fwd / rev on the intake
        // intakePower = gamepad1.right_trigger; // - gamepad1.left_trigger;

        LLResult result = limelight.getLatestResult();
        boolean hasTarget = result != null && result.isValid();
        double tx = 0;
        double ty = 0;
        double distanceInInches = 4;

        if (hasTarget) {
            tx = result.getTx();
            ty = result.getTy();

            double angleToTarget =
                    CAMERA_MOUNT_ANGLE + ty;
            // TODO: check your april tag square size in limelight calibration (local.limelight)
            distanceInInches =
                    (TARGET_HEIGHT_IN - CAMERA_HEIGHT_IN)
                            / Math.tan(Math.toRadians(angleToTarget));

            if (distanceInInches > 75) {
                goalX = -2.7;
            } else if (distanceInInches < 75) {
                goalX = -3;
            }

            error = tx - goalX;
        }

        // === Auto Align ===
        if (gamepad1.left_trigger > 0.5) {
            if (hasTarget) {
                // ===== LOCK-ON MODE =====

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = error * kp; // calc pTerm

                    // derivivative term calculations
                    curTime = getRuntime();
                    double deltaTime = curTime - lastTime;
                    double dTerm = ((error - lastError) / deltaTime) * kd;
                    rotate = Range.clip(pTerm + dTerm, -0.6, 0.6);
                    lastError = error;
                    lastTime = curTime;
                }
            }
        } else {
            // stops a derivative "spike" from stale data
            lastError = 0;
            lastTime = getRuntime();
        }


        targetRPM = getIdealRPM(distanceInInches);

        if (gamepad1.y) flywheelActived = true;
        if (gamepad1.b) flywheelActived = false;

        if (flywheelActived) {
            if (gamepad1.right_bumper) {
                targetRPM = 1800;
            }
            launcher.setVelocity(targetRPM);
        } else {
            launcher.setVelocity(0);
        }


        // ============== ACTUATE SUBSYSTEMS ==================

        launch(gamepad1.right_trigger > 0.5); // if right trigger is down, shoot a ball
        drive.driveRobotOrientated(forward, strafe, rotate); // command mecanum motors to drive

        // TODO: figure out how to control your intake
        //intake.setPower(intakePower);

        // ============== TELEMETRY ==================
        telemetry.addData("Has Target", hasTarget);
        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.addData("Distance (in)", distanceInInches);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Goal X", goalX);
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)", gamepad1.left_trigger, gamepad1.right_trigger);
    }

    @Override
    public void stop() {
    }



    void launch(boolean shotRequested) {

        if (shotRequested) {
            launcher.setVelocity(targetRPM);
        } else {
            launcher.setVelocity(0);
            return; // skip the rest of the function
        }

        // Only feed once the launcher is up to speed.
        if (launcher.getVelocity() > targetRPM - 50 && launcher.getVelocity() < targetRPM + 50) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else {
            windmillServo.setPower(0);
        }

        dashboardTelemetry.addData("Launcher vel", "%.0f / %.0f", launcher.getVelocity(), targetRPM);
        dashboardTelemetry.addData("Target Velo", targetRPM);
        dashboardTelemetry.addData("Current Velo", launcher.getVelocity());
        dashboardTelemetry.addData("KP", KP);
        dashboardTelemetry.addData("KI", KI);
        dashboardTelemetry.addData("KD", KD);
        dashboardTelemetry.addData("KF", KF);

        telemetry.addData("Launcher vel", "%.0f / %.0f", launcher.getVelocity(), targetRPM);
        telemetry.addData("Target Velo", targetRPM);
        telemetry.addData("Current Velo", launcher.getVelocity());
        telemetry.addData("KP", KP);
        telemetry.addData("KI", KI);
        telemetry.addData("KD", KD);
        telemetry.addData("KF", KF);

        dashboardTelemetry.update();
    }

    public double getIdealRPM(double distanceInInches) {

        //TODO: update this to be relevant for biobuzz - 24 nov
        double d0 = 107, rpm0 = 1250;
        double d1 = 110, rpm1 = 1250.0;
        double d2 = 113, rpm2 = 1250.0;
        double d3 = 117, rpm3 = 1250.0;
        double d4 = 120, rpm4 = 1250.0;
        double d5 = 123, rpm5 = 1250.0;
        double d6 = 126, rpm6 = 1250.0;

        double rpm;


        // linear interpolate between 2 values
        if (distanceInInches <= d0) rpm = rpm0;
        else if (distanceInInches <= d1)
            rpm = rpm0 + ((rpm1 - rpm0) / (d1 - d0)) * (distanceInInches - d0);
        else if (distanceInInches <= d2)
            rpm = rpm1 + ((rpm2 - rpm1) / (d2 - d1)) * (distanceInInches - d1);
        else if (distanceInInches <= d3)
            rpm = rpm2 + ((rpm3 - rpm2) / (d3 - d2)) * (distanceInInches - d2);
        else if (distanceInInches <= d4)
            rpm = rpm3 + ((rpm4 - rpm3) / (d4 - d3)) * (distanceInInches - d3);
        else if (distanceInInches <= d5)
            rpm = rpm4 + ((rpm5 - rpm4) / (d5 - d4)) * (distanceInInches - d4);
        else if (distanceInInches <= d6)
            rpm = rpm5 + ((rpm6 - rpm5) / (d6 - d5)) * (distanceInInches - d5);
        else
            rpm = rpm6;
        return rpm;
    }
}