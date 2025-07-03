package org.firstinspires.ftc.teamcode.sybsystems;

import java.util.function.DoubleSupplier;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;

@Config
public class Joint extends SubsystemBase {

    private final Telemetry telemetry;
    private final DcMotorEx slaveMotor;
    private final Encoder encoder;
    private final Constraints lowConstraints;
    private final Constraints highConstraints;
    private double setPoint;
    private DoubleSupplier setPointSupplier;
    private TrapezoidProfile profile;
    private ElapsedTime timer;

    private PIDController pid = new PIDController(
            Constants.ShoulderConstants.KP,
            Constants.ShoulderConstants.KI,
            Constants.ShoulderConstants.KD
    );

    public Joint(Telemetry telemetry, HardwareMap hardwareMap) {
        timer = new ElapsedTime();
        this.telemetry = telemetry;

        lowConstraints = new TrapezoidProfile.Constraints(Constants.ShoulderConstants.MAX_VELOCITY, Constants.ShoulderConstants.ACCELERATION);
        highConstraints = new Constraints(Constants.ShoulderConstants.MAX_VELOCITY, Constants.ShoulderConstants.ACCELERATION_HIGH);

        slaveMotor = hardwareMap.get(DcMotorEx.class, Constants.ShoulderConstants.SLAVE_MOTOR);
        slaveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slaveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slaveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoder = new OverflowEncoder(new RawEncoder(slaveMotor));
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);

        pid.setTolerance(Constants.ShoulderConstants.TOLERANCE);
        setPointSupplier = () -> pid.getSetPoint();
        profile = new TrapezoidProfile(lowConstraints, new State(0, 0), new State(0, 0));
        setPoint = 100;
    }

    public Command setSetPoint(double setPoint) {
        return new InstantCommand(()-> {
            this.setPoint = setPoint;
            profile = new TrapezoidProfile(lowConstraints, new State(setPoint, 0), new State(encoder.getPositionAndVelocity().position, 0));
            timer = new ElapsedTime();
        }, this);
    }

    public Command defaultCommand() {
        return setSetPoint(Constants.ShoulderConstants.ROBOT_START_ANGLE)
               .andThen(new WaitUntilCommand(() -> false));
    }

    public Command add(int add) {
        return new InstantCommand(() -> {
            double currentSetPoint = this.setPoint;
            setSetPoint(currentSetPoint + add);
        });
    }
    
    public void update() {
        pid.setSetPoint(profile.calculate(timer.seconds()).position);

        double output = pid.calculate(encoder.getPositionAndVelocity().position);

        double angle = (Constants.ShoulderConstants.ONE_TICK * encoder.getPositionAndVelocity().position) + Constants.ShoulderConstants.STARTING_ANGLE;
        double ff = Math.sin(Math.toRadians(angle)) * Constants.ShoulderConstants.kG;
        double staticPower = Constants.ShoulderConstants.kS * Math.signum(pid.getPositionError());


        slaveMotor.setPower(output + ff + Constants.ShoulderConstants.kS );

        telemetry.addData("Shoulder Power", output + ff  + staticPower);
        telemetry.addData("Shoulder FF", ff);
        telemetry.addData("Shoulder STATIC", staticPower);
        telemetry.addData("Shoulder Angle", angle);
        telemetry.addData("Current Amps", slaveMotor.getCurrent(CurrentUnit.AMPS));
    }

    @Override
    public void periodic() {
        update();

        telemetry.addData("Shoulder SetPoint", pid.getSetPoint());
        telemetry.addData("Shoulder Position", encoder.getPositionAndVelocity().position);
    }

}
