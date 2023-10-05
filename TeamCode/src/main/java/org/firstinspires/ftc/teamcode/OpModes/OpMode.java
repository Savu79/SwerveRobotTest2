package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SWERVE.HardwareSwerve;
import org.firstinspires.ftc.teamcode.SWERVE.Subsystems.DriveTrain;

@TeleOp(name = "OpMode General")
public class OpMode extends CommandOpMode {

    private HardwareSwerve robot = HardwareSwerve.getInstance();
    private DriveTrain drive;
    double loopTime=0;
    //int servonr=0;
    //double[] offSetValue= new double[4];


    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.enabled = true;
        robot.init(hardwareMap, telemetry);

        drive=new DriveTrain(robot);
    }


    @Override
    public void run(){
        //am creeat o poitie cu toate cele 3 valori ridicate la a 3-a, ca sa fie mai smooth miscarea)
        //are numele asa lung pt ca am folosit Pose2d de la RR, nu de la FTCLib
        Pose2d Pose = new Pose2d(Math.pow(gamepad1.left_stick_x, 3), Math.pow(-gamepad1.left_stick_y, 3), Math.pow(gamepad1.right_stick_x, 3));

        CommandScheduler.getInstance().run();
        robot.read(drive);
        robot.write(drive);
        robot.loop(Pose, drive);// in loop punem pozitia la care vrem sa mearga robotul


        /*if(gamepad1.dpad_left) servonr--;
        if(gamepad1.dpad_right) servonr++;
        servonr= Range.clip(servonr,0,3);
        if(gamepad1.right_stick_button)
        {
            if(gamepad1.right_trigger!=0)
                offSetValue[servonr]+=gamepad1.right_trigger/10;
            if(gamepad1.left_trigger!=0)
                offSetValue[servonr]-=gamepad1.left_trigger/10;
        }
        offSetValue[servonr]=Range.clip(offSetValue[servonr],0,10);*/


        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        //telemetry.addData("SelectedServo: ", servonr);
        //telemetry.addData("offSet servo: ", offSetValue);
        telemetry.addData("X: ", Pose.position.x);
        telemetry.addData("Y: ", Pose.position.y);
        telemetry.addData("H: ", Pose.heading);
        telemetry.update();
        loopTime = loop;
        //telemetry.update();
    }
}
