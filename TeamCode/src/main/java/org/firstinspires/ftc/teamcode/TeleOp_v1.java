package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOp_v1 extends LinearOpMode {

    double direction_y, direction_x, pivot, heading;


    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();

        Gamepad gamepad = new Gamepad();

        waitForStart();

        while(opModeIsActive()) {
//            direction_y = gamepad.left_stick_y;
//            direction_x = -gamepad.left_stick_x;
//            pivot = gamepad.right_stick_x * 0.8;
//            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (gamepad.right_bumper) {
                robot.clawClose();
            } else if (gamepad.left_bumper) {
                robot.clawOpen();
            }

            if (gamepad.circle) {
                robot.setScoringPos(0);
            } else if (gamepad.square) {
                robot.setScoringPos(1);
            }
        }
    }
}
