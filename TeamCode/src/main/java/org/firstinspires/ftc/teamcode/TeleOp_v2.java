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

        int intakeSpeed = 0;

        states state = states.IDLE;

        robot.init(hardwareMap, telemetry);
        robot.reset();

        robot.clawOpen();
        robot.setIntakePos(3);

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            pastGamepad.copy(gamepad1);

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

            // idle
            if (state == states.IDLE) {
                robot.clawOpen();
                robot.setScoringPos(0);
                robot.setIntakeSpeed(0);
                robot.setIntakePos(4);

                if (gamepad1.right_bumper && !pastGamepad.right_bumper) {
                    robot.setIntakePos(0);
                    robot.setIntakeSpeed(1);
                    state = states.INTAKE;
                }

                // intake
            } else if (state == states.INTAKE) {
                xOffset = 0.5;
                yOffset = 0.5;
                rxOffset = 0.5;
                if ((gamepad1.right_trigger > 0.2) && !(pastGamepad.right_trigger > 0.2)) {
                    robot.setIntakeSpeed(1);
                } else if (gamepad1.left_trigger > 0.2 && !(pastGamepad.left_trigger > 0.2)) {
                    robot.setIntakeSpeed(2);
                }

                if (gamepad1.right_bumper && !pastGamepad.right_bumper) {
                    robot.setIntakePos(3);
                    robot.setScoringPos(0);
                    Timer1.reset();
                    state = states.TRANSFER;

                } else if (gamepad1.left_bumper && !pastGamepad.left_bumper) {
                    state = states.IDLE;
                }

                // transfer
            } else if (state == states.TRANSFER) {
                if (Timer1.milliseconds() > 0) {
                    robot.setScoringPos(1);
                    robot.setIntakeSpeed(2);
                    state = states.SCORING;
                }

                if (gamepad1.right_bumper && !pastGamepad.right_bumper) {
                    robot.setIntakeSpeed(1);
                    state = states.SCORING;
                } else if (gamepad1.left_bumper && !pastGamepad.left_bumper) {
                    state = states.INTAKE;
                }

                // scoring
            } else if (state == states.SCORING) {
                if (gamepad1.right_trigger > 0.2) {
                    robot.clawOpen();
                } else if (gamepad1.left_trigger > 0.2) {
                    robot.clawClose();
                }

                if (gamepad1.right_bumper && !pastGamepad.right_bumper) {
                    state = states.IDLE;
                } else if (gamepad1.left_bumper && !pastGamepad.left_bumper) {
                    state = states.TRANSFER;
                }
            }

            telemetry.addData("state", state);
            telemetry.addData("horizSlider", robot.horizSlider.getCurrentPosition());
            telemetry.addData("vertSlider", robot.vertSlider.getCurrentPosition());

            telemetry.update();
        }
    }
}

