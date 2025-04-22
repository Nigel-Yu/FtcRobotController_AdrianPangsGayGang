package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOp_v1 extends LinearOpMode {

    double direction_y, direction_x, pivot, heading;


    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();

        Gamepad gamepad1 = new Gamepad();

        robot.init(hardwareMap, telemetry);

        robot.clawOpen();

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
//            direction_y = gamepad.left_stick_y;
//            direction_x = -gamepad.left_stick_x;
//            pivot = gamepad.right_stick_x * 0.8;
//            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (gamepad1.right_bumper) {
                robot.clawClose();
            } else if (gamepad1.left_bumper) {
                robot.clawOpen();
            }

            if (gamepad1.circle) {
                robot.setScoringPos(0);
            } else if (gamepad1.square) {
                robot.setScoringPos(1);
            }
        }
    }
}
