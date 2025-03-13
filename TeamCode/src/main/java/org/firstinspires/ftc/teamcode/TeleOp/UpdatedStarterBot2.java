
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="ILT TeleOp", group="Main")
//@Disabled
public class UpdatedStarterBot2 extends OpMode {

    /* Declare OpMode members. */
    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;

    DcMotor armMotor    = null;
    CRServo intake      = null;
    Servo   wrist       = null;
    Servo   armServo    = null;
    IMU     imu         = null;

    final double ARM_TICKS_PER_DEGREE = 19.7924893140647;

    final double ARM_COLLAPSED_INTO_ROBOT  = 0.5;
    final double ARM_COLLECT               = (4   * ARM_TICKS_PER_DEGREE);
    final double ARM_CLEAR_BARRIER         = (220 * ARM_TICKS_PER_DEGREE);
    final double ARM_SCORE_SPECIMEN        = (155 * ARM_TICKS_PER_DEGREE);
    final double ARM_SCORE_SAMPLE_IN_LOW   = (160 * ARM_TICKS_PER_DEGREE);
    final double ARM_ATTACH_HANGING_HOOK   = (125 * ARM_TICKS_PER_DEGREE);
    final double ARM_WINCH_ROBOT           = (15  * ARM_TICKS_PER_DEGREE);

    final double INTAKE_COLLECT    = -0.5;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    public final double WRIST_FOLDED_IN   = 0.4;
    public final double WRIST_FOLDED_OUT  = 0.13;

    final double ARM_ROTATION_COLLECT = 0.18;
    final double ARM_ROTATION_DEPOSIT = 0.86;

    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    double armPosition = (int) ARM_WINCH_ROBOT;
    double armPositionFudgeFactor;

    Gamepad gamepad1Previous = new Gamepad();
    Gamepad gamepad1Current = new Gamepad();

    Gamepad gamepad2Previous = new Gamepad();
    Gamepad gamepad2Current = new Gamepad();

    final double DRIVE_SPEED = 0.75;


    @Override
    public void init() {

        /* Define and Initialize Motors */
        frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        backRightMotor = hardwareMap.dcMotor.get("motorBR");


        armMotor = hardwareMap.get(DcMotor.class, "arm"); //the arm motor

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armServo = hardwareMap.get(Servo.class, "rotate");

        intake.setPower(INTAKE_OFF);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);
    }

    @Override
    public void start() {
        wrist.setPosition(WRIST_FOLDED_OUT);
        armServo.setPosition(ARM_ROTATION_COLLECT);
        telemetry.addLine("Starting");
    }

    @Override
    public void loop() {

        telemetry.addLine("Looping");

        gamepad1Previous.copy(gamepad1Current);
        gamepad1Current.copy(gamepad1);

        gamepad2Previous.copy(gamepad2Current);
        gamepad2Current.copy(gamepad2);

        double y = 0;
        double x = 0;
        double rx = 0;

        double gamepadActiveValue = 0.1;

        boolean gamepad2LeftStickActive = gamepad2Current.left_stick_y > gamepadActiveValue || gamepad2Current.left_stick_y < -gamepadActiveValue || gamepad2Current.left_stick_x > gamepadActiveValue || gamepad2Current.left_stick_x < -gamepadActiveValue;
        boolean gamepad2RightStickActive = gamepad2Current.right_stick_x > 0.05 || gamepad2Current.right_stick_x < -gamepadActiveValue;

        //Drive Code
        if (gamepad2LeftStickActive || gamepad2RightStickActive) {
            y = gamepad2Current.left_stick_y; // Remember, Y stick value is reversed
            x = -gamepad2Current.left_stick_x;
            rx = -gamepad2Current.right_stick_x;
        } else {
            y = gamepad1Current.left_stick_y; // Remember, Y stick value is reversed
            x = -gamepad1Current.left_stick_x;
            rx = -gamepad1Current.right_stick_x;
        }

        final double rotationFudge = 0.2;

        if (gamepad1Current.dpad_right || gamepad2Current.dpad_right) {
            if (rx <= 1 - rotationFudge) {
                rx += rotationFudge;
            }
        }

        if (gamepad1Current.dpad_left || gamepad2Current.dpad_left) {
            if (rx >= -1 + rotationFudge) {
                rx -= rotationFudge;
            }
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        if (gamepad1Current.start) {
            imu.resetYaw();
        }

//        if (gamepad2Current.start) {
//            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            armPosition = 0;
//        }

//        if (gamepad2Current.left_bumper) {
//            armPosition -= 3 * ARM_TICKS_PER_DEGREE;
//        }
//
//        if (gamepad2Current.right_bumper) {
//            armPosition += 3 * ARM_TICKS_PER_DEGREE;
//        }

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower * DRIVE_SPEED);
        backLeftMotor.setPower(backLeftPower * DRIVE_SPEED);
        frontRightMotor.setPower(frontRightPower * DRIVE_SPEED);
        backRightMotor.setPower(backRightPower * DRIVE_SPEED);

        double increaseArmPositionFudgeFactor = 0;
        double decreaseArmPositionFudgeFactor = 0;

        if (gamepad1Current.y || gamepad1Current.dpad_down) {
            decreaseArmPositionFudgeFactor = 1;
        }

        if (gamepad1Current.a || gamepad1Current.dpad_up) {
            increaseArmPositionFudgeFactor = 1;
        }

        if (gamepad2Current.right_trigger > 0.1) {
            increaseArmPositionFudgeFactor = gamepad2Current.right_trigger;
        }

        if (gamepad2Current.left_trigger > 0.1) {
            decreaseArmPositionFudgeFactor = gamepad2Current.left_trigger;
        }

        armPositionFudgeFactor = FUDGE_FACTOR * (increaseArmPositionFudgeFactor + (-decreaseArmPositionFudgeFactor));

        //TODO: Fix Controls
        if (gamepad1Current.b && !gamepad1Previous.b){
            /* This is the correct height to score the SPECIMEN in the HIGH RUNG */
            armPosition = ARM_SCORE_SPECIMEN;
            wrist.setPosition(WRIST_FOLDED_IN);
            armServo.setPosition(ARM_ROTATION_DEPOSIT);
            intake.setPower(INTAKE_OFF);
        }

        if ((gamepad1Current.right_bumper && !gamepad1Previous.right_bumper) || (gamepad2Current.right_bumper && !gamepad2Previous.right_bumper)){
            /* This is the intaking/collecting arm position */
            if (armPosition != ARM_CLEAR_BARRIER)  {
                armPosition = ARM_CLEAR_BARRIER;
                armServo.setPosition(ARM_ROTATION_DEPOSIT);
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_COLLECT);
            } else {
                armPosition = ARM_COLLECT;
                armMotor.setPower(0.5);
                armMotor.setTargetPosition((int) armPosition);
                armServo.setPosition(ARM_ROTATION_COLLECT);
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_OFF);
            }
        }

        else if (gamepad1Current.right_trigger > 0.1 && gamepad1Previous.right_trigger <= 0.1) {
            if (Math.abs(intake.getPower() - INTAKE_COLLECT) < 0.1) {
                intake.setPower(INTAKE_OFF);
            } else {
                intake.setPower(INTAKE_COLLECT);
            }

        }

        else if (gamepad1Current.left_bumper || gamepad2Current.left_bumper){
            /* This is the correct height to score the sample in the LOW BASKET */

            armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            armServo.setPosition(ARM_ROTATION_DEPOSIT);
            wrist.setPosition(WRIST_FOLDED_OUT);
            intake.setPower(INTAKE_OFF);
        }

        //TODO: Fix Arm Position

        else if (gamepad1Current.left_trigger > 0.1 && gamepad1Previous.left_trigger <= 0.1) {
            if (Math.abs(intake.getPower() - INTAKE_DEPOSIT) < 0.1) {
                intake.setPower(INTAKE_OFF);
            } else {
                intake.setPower(INTAKE_DEPOSIT);
            }

        }

        else if (gamepad1Current.back) {
                /* This turns off the intake, folds in the wrist, and moves the arm
                back to folded inside the robot. This is also the starting configuration */
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            armServo.setPosition(ARM_ROTATION_COLLECT);
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
        }

//        else if (gamepad1Current.dpad_right){
//            /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
//            armPosition = ARM_SCORE_SPECIMEN;
//            wrist.setPosition(WRIST_FOLDED_IN);
//        }

        if (gamepad2Current.dpad_up){
            /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
            armPosition = ARM_ATTACH_HANGING_HOOK;
            armServo.setPosition(ARM_ROTATION_DEPOSIT);
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
        }

        if (gamepad2Current.b) {
            armPosition = ARM_ATTACH_HANGING_HOOK + (20 * ARM_TICKS_PER_DEGREE);
            intake.setPower(INTAKE_OFF);
            armServo.setPosition(ARM_ROTATION_DEPOSIT);
        }

        if (gamepad2Current.dpad_down){
            /* this moves the arm down to lift the robot up once it has been hooked */
            armPosition = ARM_WINCH_ROBOT;
            intake.setPower(INTAKE_OFF);
            armServo.setPosition(ARM_ROTATION_DEPOSIT);
            wrist.setPosition(WRIST_FOLDED_IN);
        }



        /* Here we set the target position of our arm to match the variable that was selected
        by the driver.
        We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
        armMotor.setTargetPosition((int) (armPosition  +armPositionFudgeFactor));

        ((DcMotorEx) armMotor).setVelocity(1750);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /* TECH TIP: Encoders, integers, and doubles
        Encoders report when the motor has moved a specified angle. They send out pulses which
        only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This means that the
        position our arm is currently at can be expressed as a whole number of encoder "ticks".
        The encoder will never report a partial number of ticks. So we can store the position in
        an integer (or int).
        A lot of the variables we use in FTC are doubles. These can capture fractions of whole
        numbers. Which is great when we want our arm to move to 122.5°, or we want to set our
        servo power to 0.5.

        setTargetPosition is expecting a number of encoder ticks to drive to. Since encoder
        ticks are always whole numbers, it expects an int. But we want to think about our
        arm position in degrees. And we'd like to be able to set it to fractions of a degree.
        So we make our arm positions Doubles. This allows us to precisely multiply together
        armPosition and our armPositionFudgeFactor. But once we're done multiplying these
        variables. We can decide which exact encoder tick we want our motor to go to. We do
        this by "typecasting" our double, into an int. This takes our fractional double and
        rounds it to the nearest whole number.
        */

        /* Check to see if our arm is over the current limit, and report via telemetry. */
        if (((DcMotorEx) armMotor).isOverCurrent()){
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }


        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("Heading: : ", botHeading);
        telemetry.addData("armTarget: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        telemetry.addData("Wrist Position: ", wrist.getPosition());
        telemetry.update();
    }
}