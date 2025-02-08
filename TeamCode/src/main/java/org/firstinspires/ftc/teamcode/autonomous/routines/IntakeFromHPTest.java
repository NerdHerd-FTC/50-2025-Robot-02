package org.firstinspires.ftc.teamcode.autonomous.routines;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@Config
@Autonomous(name="HP Intake Test",  group="TEST")
@Disabled


public class IntakeFromHPTest extends LinearOpMode {

//    public Action HPIntake(double offset, ) {
//
//    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Action ApproachHPStation = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(28, -44), Math.toRadians(-45))
                .build();

        Action IntakeFromHP = drive.actionBuilder(new Pose2d(28, -44, Math.toRadians(-45)))
                .strafeTo(new Vector2d(34, -50))
                .build();

        Action moveToSubmersibleToScoreSubmersible = drive.actionBuilder(new Pose2d(34, -50, Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(8.5, -42), Math.toRadians(90))
                .build();

        Action reverseAndScoreInSubmersibleSlight = drive.actionBuilder(new Pose2d(8.5, -42, 90))
                .strafeTo(new Vector2d(9.5, -48))
                .build();

        Action reverseAndScoreInSubmersibleFull = drive.actionBuilder(new Pose2d(9.5, -48, 90))
                .strafeTo(new Vector2d(10.5, -50))
                .build();


        Actions.runBlocking(arm.clearGround());

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(ApproachHPStation);
        Actions.runBlocking(wrist.foldOut());
        Actions.runBlocking(new ParallelAction(
                intake.collect(),
                arm.intakeFromFloor()
        ));
        Actions.runBlocking(IntakeFromHP);
        Actions.runBlocking(new ParallelAction(
                arm.clearGround(),
                wrist.foldIn()
        ));

        Actions.runBlocking(
                new SequentialAction(
                        moveToSubmersibleToScoreSubmersible,
                        arm.scoreSpecimen(),

                        reverseAndScoreInSubmersibleSlight,
                        new ParallelAction(
                                reverseAndScoreInSubmersibleFull,
                                intake.deposit()
                        )
                )
        );
    }

}
