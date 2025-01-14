package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmWrist {
    private Servo wrist;
    public final double ARM_ROTATION_COLLECT = .6994;
    public final double ARM_ROTATION_DEPOSIT = .0294;

    public ArmWrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "rotate");
    }

    public class Intake implements Action {

        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(ARM_ROTATION_COLLECT);
            return false;
        }
    }

    public Action intake() {
        return new Intake();
    }

    public class Outake implements Action {

        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(ARM_ROTATION_DEPOSIT);
            return false;
        }
    }

    public Action outake() {
        return new Outake();
    }
}
