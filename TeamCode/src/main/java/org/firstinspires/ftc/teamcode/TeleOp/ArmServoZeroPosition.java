package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arm Zero Position", group = "Main" )
public class ArmServoZeroPosition extends LinearOpMode {

    Servo armServo = null;

    public void runOpMode() {

        armServo = hardwareMap.get(Servo.class, "rotate");

        armServo.setPosition(0.0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                armServo.setPosition(armServo.getPosition() + 1.0 / 1000);
            }


            if (gamepad1.left_bumper) {
                armServo.setPosition(armServo.getPosition() - 1.0 / 1000);
            }

            telemetry.addData("Servo degrees: : ", armServo.getPosition());
            telemetry.update();
        }

    }
}