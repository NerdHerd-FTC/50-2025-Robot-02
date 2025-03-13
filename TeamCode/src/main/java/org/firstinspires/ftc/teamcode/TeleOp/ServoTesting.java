package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="Servo Testing", group="Tests")

public class ServoTesting extends LinearOpMode {

    public CRServo  intake      = null;
    public Servo    wrist       = null;
    public Servo    rotate      = null;

    private Gamepad gamepad1Previous = new Gamepad();
    private Gamepad gamepad1Current = new Gamepad();

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");
        rotate  = hardwareMap.get(Servo.class, "rotate");

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        double wristPosition = 0.0;
        double rotatePosition = 0.0;


        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            gamepad1Previous.copy(gamepad1Current);
            gamepad1Current.copy(gamepad1);

            if(gamepad1Current.right_bumper && !gamepad1Previous.right_bumper){
                wristPosition += 0.01;
            }

            if (gamepad1Current.left_bumper && !gamepad1Previous.left_bumper) {
                wristPosition -= 0.01;
            }

            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(gamepad1.right_trigger);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-gamepad1.left_trigger);
            } else {
                intake.setPower(0);
            }

            if (gamepad1Current.dpad_left && !gamepad1Previous.dpad_left) {
                rotatePosition -= 0.01;
            }

            if (gamepad1Current.dpad_right && !gamepad1Previous.dpad_right) {
                rotatePosition += 0.01;
            }

            wrist.setPosition(wristPosition);
            rotate.setPosition(rotatePosition);

            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Arm Position", rotate.getPosition());
            telemetry.addData("Intake Power", intake.getPower());

            telemetry.update();

        }
    }
}