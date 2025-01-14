package org.firstinspires.ftc.teamcode.autonomous.routines;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
@Autonomous(name = "Multi Specimen Home",  group="TEST")
@Disabled

public class MultiSpecimenHome extends LinearOpMode {
    public Action generatePreloadAuto(MecanumDrive drive, Arm arm, Wrist wrist, Intake intake, Pose2d startPose) {
        TrajectoryActionBuilder moveToSubmersibleForPreload = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(8.5, -42))
                .endTrajectory();

        TrajectoryActionBuilder reverseAndScorePreloadSlight = moveToSubmersibleForPreload.fresh()
                .strafeTo(new Vector2d(9.5, -48))
                .endTrajectory();

        TrajectoryActionBuilder reverseAndScorePreloadFull = reverseAndScorePreloadSlight.fresh()
                .strafeTo(new Vector2d(10.5, -50))
                .endTrajectory();

        TrajectoryActionBuilder park = reverseAndScorePreloadFull.fresh()
                .strafeTo(new Vector2d(32, -36))

                .strafeToLinearHeading(new Vector2d(32, -12), Math.toRadians(270))
                .strafeTo(new Vector2d(45, -12))
                .strafeTo(new Vector2d(45, -57))
                .strafeTo(new Vector2d(50, -12))
                .strafeTo(new Vector2d(53, -12))
                .strafeTo(new Vector2d(53, -56))
                .strafeTo(new Vector2d(58, -12))
                .strafeTo(new Vector2d(61.5, -12))
                .strafeTo(new Vector2d(61.5, -56))

                .endTrajectory();

        SequentialAction auto = new SequentialAction(
                new ParallelAction(
                        arm.scoreSpecimen(),
                        moveToSubmersibleForPreload.build()
                ),
                reverseAndScorePreloadSlight.build(),
                new ParallelAction(
                        reverseAndScorePreloadFull.build(),
                        intake.deposit()
                ),
                new ParallelAction(
                        arm.clearGround(),
                        park.build()
                )
        );

        return auto;
    }

    public Action generateAfterSpecimenAuto(double offset, MecanumDrive drive, Arm arm, Wrist wrist, Intake intake, Pose2d startPose) {
        TrajectoryActionBuilder ApproachHPStation = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(28, -44), Math.toRadians(-45))
                .endTrajectory();

        TrajectoryActionBuilder IntakeFromHP = ApproachHPStation.fresh()
                .strafeTo(new Vector2d(34, -50))
                .endTrajectory();

        TrajectoryActionBuilder approachSubmersible = IntakeFromHP.fresh()
                .strafeToLinearHeading(new Vector2d(8.5 - offset, -46), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder moveToSubmersible = approachSubmersible.fresh()
                .strafeToLinearHeading(new Vector2d(8.5 - offset, -42), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder reverseAndScoreSpecimenSlight = moveToSubmersible.fresh()
                .strafeTo(new Vector2d((8.5 - offset) + 1, -48))
                .endTrajectory();

        TrajectoryActionBuilder reverseAndScoreSpecimenFull = reverseAndScoreSpecimenSlight.fresh()
                .strafeTo(new Vector2d((8.5 - offset) + 2, -50))
                .endTrajectory();

        Action auto = new SequentialAction(
                ApproachHPStation.build(),
                wrist.foldOut(),
                new ParallelAction(
                        intake.collect(),
                        arm.intakeFromFloor()
                ),
                IntakeFromHP.build(),
                new ParallelAction(
                        arm.clearGround(),
                        wrist.foldIn()
                ),
                approachSubmersible.build(),
                new SequentialAction(
                        moveToSubmersible.build(),
                        arm.scoreSpecimen(),
                        reverseAndScoreSpecimenSlight.build()
                ),
                new ParallelAction(
                        reverseAndScoreSpecimenFull.build(),
                        intake.deposit()
                )
        );

        return auto;
    }

    @Override public void runOpMode() {
        Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Action preloadAuto = generatePreloadAuto(drive, arm, wrist, intake, startPose);
        Action secondSpecimenAuto = generateAfterSpecimenAuto(2, drive, arm, wrist, intake, new Pose2d(12, -56, Math.toRadians(90)));

        TrajectoryActionBuilder home = drive.actionBuilder(new Pose2d(61.5, -56, Math.toRadians(270)))
                        .strafeToLinearHeading(new Vector2d(12, -56), Math.toRadians(90))
                                .endTrajectory();
        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(8.5 , -50, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(61, -55), Math.toRadians(90))
                        .endTrajectory();

        Actions.runBlocking(arm.clearGround());

        Actions.runBlocking(intake.collect());

        waitForStart();

        Actions.runBlocking(wrist.foldIn());

        Actions.runBlocking(
                new SequentialAction(
                        preloadAuto,
                        home.build(),
                        intake.off(),
                        secondSpecimenAuto,
                        park.build(),
                        arm.clearGround()
                )
        );


    }
}
