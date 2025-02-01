package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Motor Test", group = "Main")
public class MotorTestCode extends LinearOpMode {

    DcMotor motorToTest = null;

    @Override
    public void runOpMode(){

        motorToTest = hardwareMap.dcMotor.get("arm");

        motorToTest.setPower(0.0);
        motorToTest.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            motorToTest.setPower(0.5);
        }
    }
}
