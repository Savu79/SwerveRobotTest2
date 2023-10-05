package org.firstinspires.ftc.teamcode.SWERVE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SWERVE.Subsystems.DriveTrain;

@TeleOp(name= "OpModeSavu")
public class OpMode2 extends CommandOpMode {

    private HardwareSwerve robot = HardwareSwerve.getInstance();
    private DriveTrain drive;
    double[] offSetValue=new double[4];
    double loopTime=0;

    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.enabled=true;
        robot.init(hardwareMap, telemetry);
        drive=new DriveTrain(robot);
    }

    @Override
    public void run(){
        com.acmerobotics.roadrunner.Pose2d Pose = new com.acmerobotics.roadrunner.Pose2d(Math.pow(gamepad1.left_stick_x, 3), Math.pow(-gamepad1.left_stick_y, 3), Math.pow(gamepad1.right_stick_x, 3));
        CommandScheduler.getInstance().run();
        robot.read(drive);
        robot.write(drive);
        robot.loop(Pose, drive);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("offSet servo: ", offSetValue);
        loopTime = loop;
        //
    }
}
