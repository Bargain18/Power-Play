package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class leftTrajectory {
    //pos = 1, 2, 3
    public static TrajectorySequence getTraj(int pos, Arm arm, Claw claw, SampleMecanumDrive drive) {
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .setReversed(true)
                .lineTo(new Vector2d(27.0, 0))
                .build();

        switch (pos) {
            case 1:
                TrajectorySequence strafeLeft = drive.trajectorySequenceBuilder(traj.end())
                        .strafeTo(new Vector2d(27.0, -27.0))
                        .build();
                break;
            case 2:
                break;
            case 3:
                TrajectorySequence strafeRight = drive.trajectorySequenceBuilder(traj.end())
                        .strafeTo(new Vector2d(27.0, 27.0))
                        .build();
        }

        return traj;
    }
}
