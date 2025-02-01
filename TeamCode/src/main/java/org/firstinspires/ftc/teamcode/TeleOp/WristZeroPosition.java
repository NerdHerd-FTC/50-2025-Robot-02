package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Wrist Zero Position", group = "Main")
public class WristZeroPosition extends LinearOpMode {

    Servo wrist = null;

    public void runOpMode() {

        wrist = hardwareMap.get(Servo.class, "wrist");

        wrist.setPosition(0.0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                wrist.setPosition(wrist.getPosition() + 1.0/1000);
            }

            if (gamepad1.left_bumper) {
                wrist.setPosition(wrist.getPosition() - 1.0/1000);
            }

            telemetry.addData("Wrist degrees: :", wrist.getPosition());
            telemetry.update();

        }
    }
}