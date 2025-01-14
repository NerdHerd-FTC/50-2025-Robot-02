package org.firstinspires.ftc.teamcode.autonomous.routines;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@Config
@Autonomous(name="Driver: LEFT, Score yellow samples", group="Autonomous", preselectTeleOp="Meet 2 TeleOp")


public class StartLeftScoreYellowSamplesOnGround extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-12, -63.5, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Action moveToSubmersibleToScoreSubmersible = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-8.5, -42))
                .build();

        Action reverseAndScoreInSubmersibleSlight = drive.actionBuilder(new Pose2d(-8.5, -42, 90))
                .strafeTo(new Vector2d(-9.5, -48))
                .build();

        Action reverseAndScoreInSubmersibleFull = drive.actionBuilder(new Pose2d(-9.5, -48, 90))
                .strafeTo(new Vector2d(-10.5, -50))
                .build();

        Action park = drive.actionBuilder(new Pose2d(-10.5, -50, Math.toRadians(90)))
                .strafeTo(new Vector2d(-32, -36))

                .strafeToLinearHeading(new Vector2d(-32, -12), Math.toRadians(270 + (1e-6)))
                .strafeTo(new Vector2d(-45, -12))
                .strafeTo(new Vector2d(-52.5, -57))
                .strafeTo(new Vector2d(-43, -10))
                .strafeTo(new Vector2d(-54, -10))
                .strafeTo(new Vector2d(-53, -56))
                .strafeTo(new Vector2d(-56, -12))
                .strafeTo(new Vector2d(-60, -12))
                .strafeTo(new Vector2d(-60, -56))
                .strafeTo(new Vector2d(-60, -53))
                .strafeTo(new Vector2d(-57, -53))
                .turnTo(Math.toRadians(90))


                .build();

        SequentialAction auto = new SequentialAction(
                new ParallelAction(
                        arm.liftToSpecimen(),
                        moveToSubmersibleToScoreSubmersible
                ),
                arm.scoreSpecimen(),
                reverseAndScoreInSubmersibleSlight,
                new ParallelAction(
                        reverseAndScoreInSubmersibleFull,
                        intake.deposit()

                ),
                arm.clearGround(),
                park
        );

        Actions.runBlocking(arm.clearGround());

        Actions.runBlocking(intake.collect());

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(wrist.foldIn());
        Actions.runBlocking(auto);
        Actions.runBlocking(wrist.foldIn());
    }

}
