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
import org.firstinspires.ftc.teamcode.subsystems.ArmWrist;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@Config
@Autonomous(name = "Multi Specimen", group="TEST")
@Disabled

public class MultiSpecimenV2 extends LinearOpMode {
    @Override public void runOpMode() {
        Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        ArmWrist armWrist = new ArmWrist(hardwareMap);

        TrajectoryActionBuilder moveToSubmersibleForPreload = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(8.5, -42))
                .endTrajectory();

        TrajectoryActionBuilder reverseAndScorePreloadSlight = moveToSubmersibleForPreload.fresh()
                .strafeTo(new Vector2d(9.5, -48))
                .endTrajectory();

        TrajectoryActionBuilder reverseAndScorePreloadFull = reverseAndScorePreloadSlight.fresh()
                .strafeTo(new Vector2d(10.5, -50))
                .endTrajectory();

        TrajectoryActionBuilder bringInSamples = reverseAndScorePreloadFull.fresh()
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

        SequentialAction scorePreloadAndBringInSamples = new SequentialAction(
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
                        bringInSamples.build()
                )
        );

        TrajectoryActionBuilder ApproachHumanPlayerStation = bringInSamples.fresh()
                .strafeToLinearHeading(new Vector2d(28, -44), Math.toRadians(-45))
                .endTrajectory();

        TrajectoryActionBuilder IntakeFromHumanPlayerStation = ApproachHumanPlayerStation.fresh()
                .strafeTo(new Vector2d(34, -50))
                .endTrajectory();

        TrajectoryActionBuilder approachSubmersibleForSecondSpecimen = IntakeFromHumanPlayerStation.fresh()
            .strafeToLinearHeading(new Vector2d(6.5, -46), Math.toRadians(270))
            .endTrajectory();

        TrajectoryActionBuilder reverseAndScoreSecondSpecimenSlight = approachSubmersibleForSecondSpecimen.fresh()
            .strafeToLinearHeading(new Vector2d(6.5, -42), Math.toRadians(270))
            .endTrajectory();

        

        //score
//        .strafeToLinearHeading(new Vector2d(28, -44), Math.toRadians(-45))
//        .strafeTo(new Vector2d(34, -50))
//        .strafeToLinearHeading(new Vector2d(6.5, -46), Math.toRadians(270))
//        .strafeToLinearHeading(new Vector2d(6.5, -42), Math.toRadians(270))




    }
}
