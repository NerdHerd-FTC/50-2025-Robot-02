package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Arm Tester", group = "Tests")
public class ArmTester extends OpMode {
    private DcMotorEx armMotor;

    public void init() {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop() {
        telemetry.addData("Arm Position (ticks)", armMotor.getCurrentPosition());
        telemetry.addData("Arm Position (degrees)", armMotor.getCurrentPosition() / 19.7924893140647);
    }
}
