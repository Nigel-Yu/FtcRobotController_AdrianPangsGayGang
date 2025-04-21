package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Claw Control", group = "TeleOp")
class ClawControl extends OpMode {
    private Servo clawServo;
    private static final double OPEN_POSITION = 0.9;
    private static final double CLOSE_POSITION = 0.1;

    @Override
    public void init() {
        clawServo = hardwareMap.get(Servo.class, "clawServo"); // Name match configuration file
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            clawServo.setPosition(OPEN_POSITION);
            telemetry.addData("Claw Status", "Open");
        }
        else if (gamepad1.b) {
            clawServo.setPosition(CLOSE_POSITION);
            telemetry.addData("Claw Status", "Close");
        }
        telemetry.update();
    }
}