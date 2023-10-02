package org.firstinspires.ftc.teamcode.SWERVE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SWERVE.Subsystems.DriveTrain;

@Config
public class HardwareSwerve {
    public DcMotorEx FataSt;
    public DcMotorEx FataDr;
    public DcMotorEx SpateDr;
    public DcMotorEx SpateSt;
    public CRServo ServoFataSt;
    public CRServo ServoFataDr;
    public CRServo ServoSpateSt;
    public CRServo ServoSpateDr;
    public AnalogInput EncoderFataDr;
    public AnalogInput EncoderFataSt;
    public AnalogInput EncoderSpateDr;
    public AnalogInput EncoderSpateSt;

    private static HardwareSwerve instance = null;
    public boolean enabled;
    public static double WHEEL_BASE=24;
    public static double TRACKWIDTH=17;

    public static HardwareSwerve getInstance() {
        if (instance == null) {
            instance = new HardwareSwerve();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry)
    {
        FataDr= hardwareMap.get(DcMotorEx.class, "FataDr");
        FataSt= hardwareMap.get(DcMotorEx.class, "FataSt"); FataSt.setDirection(DcMotorSimple.Direction.REVERSE);
        SpateDr= hardwareMap.get(DcMotorEx.class, "SpateDr");
        SpateSt= hardwareMap.get(DcMotorEx.class, "SpateSt"); SpateSt.setDirection(DcMotorSimple.Direction.REVERSE);

        ServoFataDr= hardwareMap.get(CRServo.class, "ServoFataDr");
        ServoFataSt= hardwareMap.get(CRServo.class, "ServoFataSt"); ServoFataSt.setDirection(DcMotorSimple.Direction.REVERSE);
        ServoSpateDr= hardwareMap.get(CRServo.class, "ServoSpateDr");
        ServoSpateSt= hardwareMap.get(CRServo.class, "ServoSpateSt"); ServoSpateSt.setDirection(DcMotorSimple.Direction.REVERSE);

        EncoderFataDr=hardwareMap.get(AnalogInput.class, "EncoderFataDr");
        EncoderFataSt=hardwareMap.get(AnalogInput.class, "EncoderFataSt");
        EncoderSpateDr=hardwareMap.get(AnalogInput.class, "EncoderSpateDr");
        EncoderSpateSt=hardwareMap.get(AnalogInput.class, "EncoderSpateSt");

    }

    public void loop(Pose2d Pose, DriveTrain drive){
        try {
            if (Pose != null) {
                drive.calculeaza(Pose);
            }
            drive.updateAllModules();
        } catch (Exception ignored) {}
    }

    public void read(DriveTrain drive){
        try {
            drive.read();
        } catch (Exception ignored){}
    }

    public void write(DriveTrain drive){
        try{
            drive.write();
        }catch(Exception ignored){}
    }

    public void reset(DriveTrain drive) {

    }

    public void testServo(DriveTrain drive, boolean poz) {
        try {
            if (poz) drive.testPozServo1();
            else drive.testPozServo2();
        } catch (Exception ignored){}
    }
    public void testMotors(DriveTrain drive, boolean on) {
        try {
            drive.testMotors(on);
        } catch (Exception ignored){}
    }
}
