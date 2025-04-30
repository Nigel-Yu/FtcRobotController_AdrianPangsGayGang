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
    DcMotor vertSlider, horizSlider;
    Servo claw, clawPitchL, clawPitchR, armL, armR, intakePitchL, intakePitchR;
    CRServo rollerL, rollerR;
    IMU imu;

    Telemetry telemetry;

    int intakeSpeed = 0;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        vertSlider = hardwareMap.get(DcMotor.class, "vertSliderd");
        horizSlider = hardwareMap.get(DcMotor.class, "horizSlider");
        claw = hardwareMap.get(Servo.class, "claw");
        clawPitchL = hardwareMap.get(Servo.class, "clawPitchL");
        clawPitchR = hardwareMap.get(Servo.class, "clawPitchR");
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        rollerL = hardwareMap.get(CRServo.class, "intakeL");
        rollerR = hardwareMap.get(CRServo.class, "intakeR");
        intakePitchL = hardwareMap.get(Servo.class, "intakePitchL");
        intakePitchR = hardwareMap.get(Servo.class, "intakePitchR");


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

        horizSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        vertSlider.setDirection(DcMotorSimple.Direction.FORWARD);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vertSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vertSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rollerL.setDirection(CRServo.Direction.FORWARD);
        rollerR.setDirection(CRServo.Direction.REVERSE);

        intakePitchL.setDirection(Servo.Direction.FORWARD);
        intakePitchR.setDirection(Servo.Direction.REVERSE);

        imu.resetYaw();
    }

    public void clawOpen() {
        claw.setPosition(0);
    }

    public void clawClose() {
        claw.setPosition(0.2);
    }

    private void setHorizSlider(int pow, int pos) {
        horizSlider.setPower(pow);
        horizSlider.setTargetPosition(pos);
        horizSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setVertSlider(int pow, int pos) {
        vertSlider.setPower(pow);
        vertSlider.setTargetPosition(pos);
        vertSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void setArm(int pos) {
        armL.setPosition(0);
        armR.setPosition(0);
    }

    private void setClawPitch(int pos) {
        clawPitchR.setPosition(0);
        clawPitchL.setPosition(0);
    }


    public void setIntakeSpeed() {
        switch (intakeSpeed) {
            case 0: // stop
                rollerL.setPower(0);
                rollerR.setPower(0);
                intakeSpeed = 0;
                break;
            case 1: // in
                rollerL.setPower(1);
                rollerR.setPower(1);
                intakeSpeed = 1;
                break;
            case 2: // out
                rollerL.setPower(-1);
                rollerR.setPower(-1);
                intakeSpeed = 2;
        }
    }


    public void setIntakePos(int pos) {
        switch (pos) {
            case 0: // intake close
                intakePitchL.setPosition(0.67);
                intakePitchR.setPosition(0.67);
                setHorizSlider(1,0);
                break;
            case 1: // intake mid
                intakePitchL.setPosition(0.67);
                intakePitchR.setPosition(0.67);
                setHorizSlider(1,0);
                break;
            case 2: // intake far
                intakePitchL.setPosition(0.67);
                intakePitchR.setPosition(0.67);
                setHorizSlider(1,0);
                break;
            case 3: // transfer
                // TODO: find transfer pos
                intakePitchL.setPosition(0);
                intakePitchR.setPosition(0);
                setHorizSlider(1,0);
            case 4: // idle
                intakePitchL.setPosition(0);
                intakePitchR.setPosition(0);
                setHorizSlider(1,0);
        }
    }

    // TODO: find scoring pos
    public void setScoringPos(int pos) {
        switch (pos) {
            case 0: // transfer
                clawOpen();
                setClawPitch(0);
                setArm(0);
                setVertSlider(1,0);
                break;
            case 1: // scoring high
                clawClose();
                setClawPitch(0);
                setArm(0);
                setVertSlider(1,0);
                break;
            case 2: // scoring mid
                clawClose();
                setClawPitch(0);
                setArm(0);
                setVertSlider(1,0);
                break;
            case 3: // idle
                clawOpen();
                setClawPitch(0);
                setArm(0);
                setVertSlider(1,0);
        }
    }


}
