package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Timer;

@TeleOp
public class TeleOp_v2 extends LinearOpMode {

    public enum states {
        IDLE,
        INTAKE,
        TRANSFER,
        SCORING
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Project1Hardware robot = new Project1Hardware();

        // TODO: find optimal offset
        double yOffset = 0.5;
        double xOffset = 0.5;
        double rxOffset = 0.5;

        Gamepad pastGamepad = new Gamepad();

        ElapsedTime Timer1 = new ElapsedTime();


        states state = states.IDLE;

        robot.init(hardwareMap, telemetry);
        robot.reset();

        robot.clawOpen();
        robot.setScoringPos("idle");
        robot.setIntakePos("idle");
        robot.intakeSpeed = 0;

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pastGamepad.copy(gamepad1);

            // drivetrain
            double y = -gamepad1.left_stick_y * yOffset;
            double x = gamepad1.left_stick_x * xOffset;
            double rx = gamepad1.right_stick_x * rxOffset;

            double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            //rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            robot.FL.setPower((rotY + rotX + rx) / denominator);
            robot.BL.setPower((rotY - rotX + rx) / denominator);
            robot.FR.setPower((rotY - rotX - rx) / denominator);
            robot.BR.setPower((rotY + rotX - rx) / denominator);

            if (gamepad1.options) {
                robot.reset();
            }

            robot.setIntakeSpeed();

            // idle
            if (state == states.IDLE) {
                robot.clawOpen();
                robot.setScoringPos("idle");
                robot.setIntakePos("idle");
                robot.intakeSpeed = 0;

                if (gamepad1.right_bumper && !pastGamepad.right_bumper) {
                    robot.setIntakePos("close");
                    robot.intakeSpeed = 1;
                    state = states.INTAKE;
                }

                // intake
            } else if (state == states.INTAKE) {
                // TODO: find optimal offset
                xOffset = 0.5;
                yOffset = 0.5;
                rxOffset = 0.5;
                if (gamepad1.right_trigger > 0.2) {
                    robot.intakeSpeed = 1;
                } else if (gamepad1.left_trigger > 0.2) {
                    robot.intakeSpeed = 2;
                } else if (gamepad1.dpad_down) { // failsafe
                    robot.intakeSpeed = 0;
                }

                if (gamepad1.square) { //X
                    robot.setIntakePos("close");
                } else if (gamepad1.cross) { //A
                    robot.setIntakePos("mid");
                } else if (gamepad1.circle) { //B
                    robot.setIntakePos("far");
                }


                if (gamepad1.right_bumper && !pastGamepad.right_bumper) {
                    robot.setIntakePos("transfer");
                    robot.setScoringPos("transfer");
                    Timer1.reset();
                    state = states.TRANSFER;
                } else if (gamepad1.left_bumper && !pastGamepad.left_bumper) {
                    state = states.IDLE;
                }

                // transfer
            } else if (state == states.TRANSFER) {
                if (Timer1.milliseconds() > 50) {
                    robot.clawClose();
                    robot.intakeSpeed = 2;
                    robot.setScoringPos("scoring");
                    state = states.SCORING;
                }

                // failsafe
                if (gamepad1.right_bumper && !pastGamepad.right_bumper) {
                    robot.intakeSpeed = 0;
                    robot.setScoringPos("scoring");
                    robot.setIntakePos("idle");
                    state = states.SCORING;
                } else if (gamepad1.left_bumper && !pastGamepad.left_bumper) {
                    robot.intakeSpeed = 1;
                    robot.setIntakePos("close");
                    robot.setScoringPos("idle");
                    state = states.INTAKE;
                }

                // scoring
            } else if (state == states.SCORING) {
                if (gamepad1.right_trigger > 0.2) {
                    robot.clawOpen();
                } else if (gamepad1.left_trigger > 0.2) { // failsafe
                    robot.clawClose();
                }

                if (gamepad1.cross) { //A
                    robot.setScoringPos("low");
                } else if (gamepad1.circle) { //B
                    robot.setScoringPos("high");
                }

                if (gamepad1.right_bumper && !pastGamepad.right_bumper) {
                    state = states.IDLE;
                }
            }

            telemetry.addData("state", state);
            telemetry.addData("horizSlider", robot.horizSlider.getCurrentPosition());
            telemetry.addData("vertSlider", robot.vertSlider.getCurrentPosition());
            telemetry.addData("intakeSpeed", robot.intakeSpeed);

            telemetry.update();
        }
    }
}

