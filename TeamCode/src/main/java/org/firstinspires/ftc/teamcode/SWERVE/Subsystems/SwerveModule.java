package org.firstinspires.ftc.teamcode.SWERVE.Subsystems;

;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;


@Config
public class SwerveModule {
    public static double Ps = 0, Is = 0, Ds = 0; //CONTROL PID PT SERVO
    //public static double Pm = 0, Im = 0, Dm = 0;
    public static double K_STATIC = 0; //
    public static double MAX_SERVO = 1, MAX_MOTOR = 1;  // PUTERE MAXIMA ADMISA PT SERVO SI MOTOR

    public static boolean MOTOR_FLIPPING = true;  //DACA MOTORUL TRECE DE 180 DE GRADE

    public static double WHEEL_RADIUS = 1.4; // in
    public static double GEAR_RATIO = 1 / (3.5 * 1.5 * 2); // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 28;  //MOTOR
    private double VoltageRotation=0;
    private int voltageRot=0;

    private DcMotorEx motor; //MOTORUL CU ENCODER
    private CRServo servo; //SERVOUL
    private AnalogInput encoder; //ENCODER SERVO
    private PIDFController rotationController; //CONTROLLER PIDF
    //private PIDFController speedController;

    public boolean wheelFlipped = false;
    private double targetS = 0.0;
    //private double targetM = 0.0;
    private double positionS = 0.0;
    //private double positionM = 0.0;
    private double lastVoltage;
    private double voltage;
    boolean creste=true;


    public SwerveModule(DcMotorEx m, CRServo s, AnalogInput e) {  //DECLARAM UN MODUL CU MOTOR, SERVO SI ENCODER PT SERVO
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //speedController = new PIDFController(Pm, Im, Dm, 0);

        servo = s;
        ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));

        encoder = e;
        rotationController = new PIDFController(Ps, Is, Ds, 0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public SwerveModule(HardwareMap hardwareMap, String mName, String sName, String eName) {
        this(hardwareMap.get(DcMotorEx.class, mName),
                hardwareMap.get(CRServo.class, sName),
                hardwareMap.get(AnalogInput.class, eName));
    }

    public void read() {
        voltage = encoder.getVoltage();}



    public void update() {

        rotationController.setPIDF(Ps, Is, Ds, 0);
        double targetS = getTargetRotation(), currentS = getModuleRotation(voltage);
        double error = targetS - currentS;
        if (MOTOR_FLIPPING && error > 90) {
            targetS = targetS - 180;
            creste=false;
            wheelFlipped = true;

        } else if(MOTOR_FLIPPING && error<90 && error>0) {
            creste=true;
            wheelFlipped = false;

        } else if(MOTOR_FLIPPING && error <-90) {
            targetS=targetS+180;
            creste=true;
            wheelFlipped=true;

        } else if(MOTOR_FLIPPING && error>-90 && error<0) {
            creste=false;
            wheelFlipped=false;
        }

        error = normalizeRadians(targetS - currentS);

        double power = Range.clip(rotationController.calculate(0, error), -MAX_SERVO, MAX_SERVO);
        if (Double.isNaN(power)) power = 0;
        servo.setPower(power);
        if(creste && (voltage-lastVoltage)<0) {
            VoltageRotation+=3.3;
        }
        if(!creste && (voltage-lastVoltage)>0) {
            VoltageRotation-=3.3;
        }
        if (Math.abs(VoltageRotation)==13.2) {
            VoltageRotation=0;
        }
        lastVoltage=voltage;

        //speedController.setPIDF(Pm, Im, Dm, 0);
        //double targetM=, currentM=motor.getCurrentPosition();

    }

    public double getTargetRotation() {
        return targetS;
    }

    public double getModuleRotation(double Voltage) {
        return (VoltageRotation+Voltage)/4.4*360;
    }

    public void setMotorPower(double power) {
        if (wheelFlipped) power *= -1;
        lastMotorPower = power;
        motor.setPower(power);
    }

    public void setTargetRotation(double target) {
        this.targetS = target;
    }

    public String getTelemetry(String name) {
        return String.format(Locale.ENGLISH, "%s: Motor Flipped: %b \ncurrent position %.2f target position %.2f flip modifer = %d motor power = %.2f", name, wheelFlipped, getModuleRotation(lastVoltage), getTargetRotation(), flipModifier(), lastMotorPower);
    }

    public int flipModifier() {
        return wheelFlipped ? -1 : 1;
    }


    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }


    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(runMode, coefficients);
    }
    public double lastMotorPower = 0;

    public double getServoPower() {
        return servo.getPower();
    }

    public double getWheelPosition() {
        return encoderTicksToInches(motor.getCurrentPosition());
    }

    public double getWheelVelocity() {
        return encoderTicksToInches(motor.getVelocity());
    }

    public SwerveModuleState asState() {
        return new SwerveModuleState(this);
    }

    public static class SwerveModuleState {
        public SwerveModule module;
        public double wheelPos, podRot;

        public SwerveModuleState(SwerveModule s) {
            module = s;
            wheelPos = 0;
            podRot = 0;
        }

        public SwerveModuleState update() {
            return setState(-module.getWheelPosition(), module.getModuleRotation(module.lastVoltage));
        }

        public SwerveModuleState setState(double wheel, double pod) {
            wheelPos = wheel;
            podRot = pod;
            return this;
        }

        //TODO add averaging for podrots based off of past values
        /*public Vector2d calculateDelta() {
            double oldWheel = wheelPos;
            update();
            return Vector2d.polar(wheelPos - oldWheel, podRot);
        }*/
    }

    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}