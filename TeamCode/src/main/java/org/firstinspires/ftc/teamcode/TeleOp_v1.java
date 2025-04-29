package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOp_v1 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();

        // TODO: find optimal offset
        double yOffset = 0.5;
        double xOffset = 0.5;
        double rxOffset = 0.5;

        robot.init(hardwareMap, telemetry);
        robot.reset();

//        robot.clawOpen();
//        robot.setIntakePos(3);

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

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




//            if (gamepad1.right_bumper) {
//                robot.clawClose();
//            } else if (gamepad1.left_bumper) {
//                robot.clawOpen();
//            }
//
//            if (gamepad1.circle) {
//                robot.setScoringPos(0);
//            } else if (gamepad1.square) {
//                robot.setScoringPos(1);
//            }
//
//            if (gamepad1.triangle) {
//                robot.setIntakePos(0);
//            } else if (gamepad1.cross) {
//                robot.setIntakePos(1);
//            }
//
//            if (gamepad1.dpad_down) {
//                robot.setIntakeSpeed(0);
//            } else if (gamepad1.dpad_right) {
//                robot.setIntakeSpeed(1);
//            } else if (gamepad1.dpad_left) {
//                robot.setIntakeSpeed(2);
//            }
        }
    }
}
