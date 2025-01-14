package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private CRServo intake;
    final double INTAKE_COLLECT    = -0.5;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(CRServo.class, "intake");

        intake.setPower(INTAKE_OFF);
    }

    public class Collect implements Action {

        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(INTAKE_COLLECT);
            return false;
        }
    }

    public Action collect() {
        return new Collect();
    }

    public class Deposit implements Action {

        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(INTAKE_DEPOSIT);
            return false;
        }
    }

    public Action deposit() {
        return new Deposit();
    }

    public class Off implements Action {

        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(INTAKE_OFF);
            return false;
        }
    }

    public Action off() {
        return new Off();
    }
}
