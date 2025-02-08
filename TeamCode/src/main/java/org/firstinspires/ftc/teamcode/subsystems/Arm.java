package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private DcMotorEx arm;
    public final double ARM_TICKS_PER_DEGREE =   19.7924893140647;//; 7.46666666667//exact fraction is (194481/9826)
    public final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    public final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    public final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SPECIMEN        = 167 * ARM_TICKS_PER_DEGREE;
    public final double ARM_HOOK_SPECIMEN         = 158 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SAMPLE_IN_LOW   = 162 * ARM_TICKS_PER_DEGREE;
    public final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    public final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;
    public final double ARM_TOUCH_BAR             = 155 * ARM_TICKS_PER_DEGREE;

    public final int VELOCITY = 2500; //450

    /* A number in degrees that the triggers can adjust the arm position by */
    public final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public class LiftToSpecimen implements Action {

        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setTargetPosition((int) ARM_HOOK_SPECIMEN);
            arm.setVelocity(VELOCITY);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (Math.abs(arm.getCurrentPosition() - ARM_HOOK_SPECIMEN) < 10) {
                return false;
            } else {
                return true;
            }
        }
    }

    public class ScoreSpecimen implements Action {

        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setTargetPosition((int) ARM_SCORE_SPECIMEN);
            arm.setVelocity(VELOCITY);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (Math.abs(arm.getCurrentPosition() - ARM_SCORE_SPECIMEN) < 10) {
                return false;
            } else {
                return true;
            }
        }
    }

    public class ClearGround implements Action {

        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setTargetPosition((int) ARM_WINCH_ROBOT);
            arm.setVelocity(VELOCITY);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (Math.abs(arm.getCurrentPosition() - ARM_WINCH_ROBOT) < 10) {
                return false;
            } else {
                return true;
            }
        }
    }

    public class TouchBottomBar implements Action {

        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setTargetPosition((int) ARM_TOUCH_BAR);
            arm.setVelocity(VELOCITY);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (Math.abs(arm.getCurrentPosition() - ARM_TOUCH_BAR) < 10) {
                return false;
            } else {
                return true;
            }
        }
    }

    public class IntakeFromFloor implements Action {
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            arm.setTargetPosition((int) ARM_COLLECT);
            arm.setVelocity(VELOCITY);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (Math.abs(arm.getCurrentPosition() - ARM_COLLECT) < 10) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action liftToSpecimen() {
        return new LiftToSpecimen();
    }
    public Action scoreSpecimen() {
        return new ScoreSpecimen();
    }
    public Action clearGround() {
        return new ClearGround();
    }
    public Action touchBottomBar() { return  new TouchBottomBar(); }
    public Action intakeFromFloor() { return  new IntakeFromFloor(); }
}
