package org.firstinspires.ftc.teamcode.autonomous.routines.oldStyle;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@Disabled
@Config
@Autonomous(name="Delivery", group="Autonomous")


public class DeliveryTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Action deliver = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(32, -36))

                .strafeToLinearHeading(new Vector2d(32, -12), Math.toRadians(270))
                .strafeTo(new Vector2d(45, -12))
                .strafeTo(new Vector2d(45, -60))
                .strafeTo(new Vector2d(50, -12))
                .strafeTo(new Vector2d(53, -12))
                .strafeTo(new Vector2d(53, -60))
                .strafeTo(new Vector2d(58, -12))
                .strafeTo(new Vector2d(61, -12))
                .strafeTo(new Vector2d(61, -60))
                .build();


        Actions.runBlocking(arm.clearGround());

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(deliver);
    }

}
