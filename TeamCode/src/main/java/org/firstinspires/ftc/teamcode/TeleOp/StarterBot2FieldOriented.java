package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Oriented", group = "Main")

public class StarterBot2FieldOriented extends LinearOpMode {

    DcMotor armMotor = null;
    CRServo intake = null;
    Servo wrist = null;
    Servo armServo = null;

    final double ARM_TICKS_PER_DEGREE = 19.7924893140647;

    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT              = 2 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER        = 240 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN       = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW  = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK  = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT          = 15  * ARM_TICKS_PER_DEGREE;

    final double INTAKE_COLLECT  = -1.0;
    final double INTAKE_OFF      = 0.0;
    final double INTAKE_DEPOSIT  = 0.5;

    final double WRIST_FOLDED_IN   = .5556;
    final double WRIST_FOLDED_OUT  = .2261;

    final double ARM_ROTATION_COLLECT = .6994;
    final double ARM_ROTATION_DEPOSIT = .0294;

    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    double armPosition = 0.0;
    double armPositionFudgeFactor;

    @Override
    public void runOpMode(){

        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armServo = hardwareMap.get(Servo.class, "rotate");

        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_OUT);
        armMotor.setTargetPosition((int) ARM_SCORE_SAMPLE_IN_LOW);
        armServo.setPosition(ARM_ROTATION_COLLECT);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            if(gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX +rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.a){
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad1.x){
                intake.setPower(INTAKE_OFF);
            }
            else if (gamepad1.b){
                intake.setPower(INTAKE_DEPOSIT);
            }

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));

            if(gamepad1.right_bumper){
                armPosition = ARM_COLLECT;
                armMotor.setPower(0.5);
                armMotor.setTargetPosition((int) armPosition);
                armServo.setPosition(ARM_ROTATION_COLLECT);
                wrist.setPosition(WRIST_FOLDED_OUT);
            }

            else if (gamepad1.left_bumper){
                armPosition = ARM_CLEAR_BARRIER;
                armMotor.setPower(0.5);
                armMotor.setTargetPosition((int) armPosition);
                wrist.setPosition(WRIST_FOLDED_OUT);
                armServo.setPosition(ARM_ROTATION_DEPOSIT);
            }

            else if (gamepad1.y){
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
                armMotor.setPower(0.5);
                armMotor.setTargetPosition((int) armPosition);
                wrist.setPosition(WRIST_FOLDED_OUT);
                armServo.setPosition(ARM_ROTATION_DEPOSIT);
            }

            else if (gamepad1.dpad_left){
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                armMotor.setPower(0.5);
                armMotor.setTargetPosition((int) armPosition);
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
                armServo.setPosition(ARM_ROTATION_COLLECT);
            }

            else if (gamepad1.dpad_right){
                armPosition = ARM_SCORE_SPECIMEN;
                armMotor.setPower(0.5);
                armMotor.setTargetPosition((int) armPosition);
                wrist.setPosition(WRIST_FOLDED_IN);
                armServo.setPosition(ARM_ROTATION_DEPOSIT);
            }

            else if (gamepad1.dpad_up){
                // This sets the arm to vertical to hook onto the LOW RUNG for hanging
                armPosition = ARM_ATTACH_HANGING_HOOK;
                armMotor.setPower(0.5);
                armMotor.setTargetPosition((int) armPosition);
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
                armServo.setPosition(ARM_ROTATION_DEPOSIT);
            }

            else if (gamepad1.dpad_down){
                // This moves the arm down to lift the robot up once it has been hooked
                armPosition = ARM_WINCH_ROBOT;
                armMotor.setPower(0.5);
                armMotor.setTargetPosition((int) armPosition);
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_OUT);
                armServo.setPosition(ARM_ROTATION_DEPOSIT);
            }

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

            if(((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            telemetry.addData("Heading: : ", botHeading);
            telemetry.addData("Servo degrees: : ", armServo.getPosition());
            telemetry.addData("Wrist degrees: :", wrist.getPosition());
            telemetry.addData("armTarget", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder", armMotor.getCurrentPosition());
            telemetry.addData("imu", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.update();

        }
    }
}
