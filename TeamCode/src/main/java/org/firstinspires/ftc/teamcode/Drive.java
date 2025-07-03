package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {

    private final Telemetry telemetry;

    private final Motor RB;
    private final Motor FL;
    private final Motor FR;
    private final Motor LB;

    private final MecanumDrive mecanum;
    private int setPointCount = 0;

    private final LazyImu imu;

    private PIDController pid = new PIDController(
            DriveConstants.KP,
            DriveConstants.KI,
            DriveConstants.KD
    );


    public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        imu = new LazyHardwareMapImu(hardwareMap, "imu",
        new RevHubOrientationOnRobot(DriveConstants.LOGO_DIRECTION,
        DriveConstants.USB_DIRECTION));

        FR = new Motor(hardwareMap, DriveConstants.MOTOR_FR);
        FR.setRunMode(Motor.RunMode.RawPower);
        FR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FR.setInverted(true);

        LB = new Motor(hardwareMap, DriveConstants.MOTOR_LB);
        LB.setRunMode(Motor.RunMode.RawPower);
        LB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        LB.setInverted(false);

        RB = new Motor(hardwareMap, DriveConstants.MOTOR_RB);
        RB.setRunMode(Motor.RunMode.RawPower);
        RB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        RB.setInverted(true);

        FL = new Motor(hardwareMap, DriveConstants.MOTOR_FL);
        FL.setRunMode(Motor.RunMode.RawPower);
        FL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FL.setInverted(false);

        mecanum = new MecanumDrive(false, FL, FR, LB, RB);


        pid.setTolerance(DriveConstants.TOLERANCE);

    }

    public Command driveRobotCentric(DoubleSupplier y, DoubleSupplier x, DoubleSupplier rx) {
        return new RunCommand(
                () -> mecanum.driveRobotCentric(
                        x.getAsDouble(),
                        y.getAsDouble(),
                        rx.getAsDouble()

                ), this
        );

    }

    public Command driveFieldCentric(DoubleSupplier y, DoubleSupplier x, DoubleSupplier rx) {
        return new RunCommand(
                () -> mecanum.driveFieldCentric(
                        x.getAsDouble(),
                        y.getAsDouble(),
                        rx.getAsDouble(),
                        imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)

                ), this
        );

    }

    public Command setDriveAngle(double setPoint) {
        return new RunCommand(
                () -> {
                    pid.setSetPoint(setPoint);
                    double power = pid.calculate(imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    mecanum.driveRobotCentric(0, 0, power);
                    if (pid.atSetPoint()) {
                        setPointCount++;
                    } else {
                        setPointCount = 0;
                    }
                }, this).interruptOn(() -> isAtSetPoint() && pid.getSetPoint() == setPoint
        ).withTimeout(1000);

    }

    public Command rotateToWall() {
        return setDriveAngle(DriveConstants.WALL_ROTATE);
    }

    public boolean isAtSetPoint() {
        return setPointCount == DriveConstants.MAX_SET_POINT_COUNT;
    }

    public Command resetIMU() {
        return new InstantCommand(() -> imu.get().resetYaw());

    }

    public Command takeFromObservation() {
        return new RunCommand(() -> setDriveAngle(DriveConstants.INTAKE_FROM_OBSERVATION));
    }

    public Command autoStartPosition() {
        return new RunCommand(() -> setDriveAngle(DriveConstants.START_AUTO_AGAIN));
    }


    @Override
    public void periodic() {
        telemetry.addData("Heading", imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Set Point", pid.getSetPoint());
        telemetry.addData("is At Set Point", pid.atSetPoint());
        pid.calculate(imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

    }

}
