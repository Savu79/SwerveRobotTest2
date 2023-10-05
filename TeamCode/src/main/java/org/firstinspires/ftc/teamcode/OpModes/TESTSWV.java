package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SWERVE.HardwareSwerve;
import org.firstinspires.ftc.teamcode.SWERVE.Subsystems.DriveTrain;

@TeleOp(name = "Test Swerve")
@Config
public class TESTSWV extends CommandOpMode {

    private HardwareSwerve robot = HardwareSwerve.getInstance();
    private DriveTrain drive;
    double loopTime = 0;
    boolean poz = true;
    ElapsedTime timer= new ElapsedTime();
    //int servonr=0;
    //double[] offSetValue= new double[4];
    //


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.enabled = true;
        robot.init(hardwareMap, telemetry);

        drive = new DriveTrain(robot);
    }


    @Override
    public void run() {

        CommandScheduler.getInstance().run();
        robot.testServo(drive, poz);
        robot.testMotors(drive, gamepad1.a); // daca apaesi a se pornesc motoarele
        if (timer.seconds() > 5) {
            poz = !poz;
            timer.reset();
        }

    }

}