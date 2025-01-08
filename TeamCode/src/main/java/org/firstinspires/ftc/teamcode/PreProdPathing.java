package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
public class PreProdPathing extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));
        int x1 = 20;
        int x2 = 20;
        int x3 = 0;
        int x4 = 10;
        int y1 = 9;
        int y2 = 20;
        int y3 = 0;
        int y4 = -8;


        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
        .splineTo(new Vector2d(x1, y1), heading)
        .splineTo(new Vector2d(x2, y2), heading)
        splineTo(new Vector2d(x3, y3), heading)
        .splineTo(new Vector2d(x4, y4), heading)
        .build();
      
        drive.followTrajectory(traj1);
  
    }
}