package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Arm Position Tuner", group="Tests")
public class ArmPositionTuner extends OpMode {
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647;  //7.46666666667; //exact fraction is (194481/9826)
    double armPositionDegrees = 0;
    DcMotorEx armMotor = null;

    private Gamepad gamepad1Previous = new Gamepad();
    private Gamepad gamepad1Current = new Gamepad();

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        gamepad1Previous.copy(gamepad1Current);
        gamepad1Current.copy(gamepad1);
        double modifier = 1;

        if (gamepad1Current.right_trigger > 0) {
            modifier = 10;
        }

        if (gamepad1Current.dpad_up && !gamepad1Previous.dpad_up) {
            armPositionDegrees += 1*modifier;
        } else if (gamepad1Current.dpad_down && !gamepad1Previous.dpad_down) {
            armPositionDegrees -= 1*modifier;
        }

        armMotor.setTargetPosition((int)(armPositionDegrees * ARM_TICKS_PER_DEGREE));
        ((DcMotorEx) armMotor).setVelocity(1750);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        telemetry.addData("Arm Position (degrees)", armPositionDegrees);
        telemetry.addData("Arm Position (ticks)", armMotor.getCurrentPosition());
        telemetry.update();
    }
}
