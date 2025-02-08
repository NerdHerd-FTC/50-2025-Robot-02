package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Intake Tuning", group = "Main")
public class IntakeTuning extends LinearOpMode {

    CRServo intake = null;

    public void runOpMode() {

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            intake = hardwareMap.get(CRServo.class, "intake");

            intake.setPower(-0.5);

            if (gamepad1.right_bumper) {
                intake.setPower(intake.getPower() + 0.1);
            }

            if (gamepad1.dpad_up) {
                intake.setPower(intake.getPower() + 0.01);
            }

            if (gamepad1.left_bumper) {
                intake.setPower(intake.getPower() - 0.1);
            }

            if (gamepad1.dpad_down) {
                intake.setPower(intake.getPower() - 0.01);
            }

            telemetry.addData("Intake Power", intake.getPower());
            telemetry.update();
        }
    }
}
