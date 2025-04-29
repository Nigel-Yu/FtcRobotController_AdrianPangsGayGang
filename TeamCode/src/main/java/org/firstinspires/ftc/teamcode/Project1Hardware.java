package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Project1Hardware {
    DcMotor FL, FR, BL, BR;
//    DcMotor vertSliderL, vertSliderR;
//    Servo claw, clawYaw, intakePitchL, intakePitchR;
//    CRServo rollerL, rollerR;
    IMU imu;

    Telemetry telemetry;


    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
//        vertSliderL = hardwareMap.get(DcMotor.class, "vertSliderL");
//        vertSliderR = hardwareMap.get(DcMotor.class, "vertSliderR");
//        claw = hardwareMap.get(Servo.class, "claw");
//        clawYaw = hardwareMap.get(Servo.class, "clawYaw");
//        rollerL = hardwareMap.get(CRServo.class, "intakeL");
//        rollerR = hardwareMap.get(CRServo.class, "intakeR");
//        intakePitchL = hardwareMap.get(Servo.class, "intakePitchL");
//        intakePitchR = hardwareMap.get(Servo.class, "intakePitchR");


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);



        this.telemetry = telemetry;
    }

    public void reset() {
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

//        rollerL.setDirection(CRServo.Direction.FORWARD);
//        rollerR.setDirection(CRServo.Direction.REVERSE);
//
//        intakePitchL.setDirection(Servo.Direction.FORWARD);
//        intakePitchR.setDirection(Servo.Direction.REVERSE);

        imu.resetYaw();
    }
//
//    public void clawOpen() {
//        claw.setPosition(0);
//    }
//
//    public void clawClose() {
//        claw.setPosition(0.2);
//    }
//
//    /**
//     *
//     * @param dir - 0 = stop, 1 = in, 2 = out
//     */
//    public void setIntakeSpeed(int dir) {
//        switch (dir) {
//            case 0: // stop
//                rollerL.setPower(0);
//                rollerR.setPower(0);
//                break;
//            case 1: // in
//                rollerL.setPower(1);
//                rollerR.setPower(1);
//                break;
//            case 2: // out
//                rollerL.setPower(-1);
//                rollerR.setPower(-1);
//        }
//    }
//
//    /**
//     *
//     * @param pos - 0 = intake, 1 = transfer, 2 = horizontal
//     */
//    public void setIntakePos(int pos) {
//        switch (pos) {
//            case 0: // intake
//                intakePitchL.setPosition(0.67);
//                intakePitchR.setPosition(0.67);
//                break;
//            case 1: // transfer
//                // TODO: find transfer pos
//                intakePitchL.setPosition(0);
//                intakePitchR.setPosition(0);
//            case 3: // horizontal
//                intakePitchL.setPosition(0);
//                intakePitchR.setPosition(0);
//        }
//    }
//
//    /**
//     *
//     * @param pos - 0 = transfer, 1 = scoring
//     */
//    public void setScoringPos(int pos) {
//        switch (pos) {
//            case 0: // transfer
//                clawOpen();
//                clawYaw.setPosition(0);
//                break;
//            case 1: // scoring
//                clawClose();
//                clawYaw.setPosition(.65);
//        }
//    }

}
