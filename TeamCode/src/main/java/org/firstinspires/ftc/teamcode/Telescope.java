package org.firstinspires.ftc.teamcode;

import java.util.function.DoubleSupplier;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Telescope  extends SubsystemBase {
        private final Telemetry telemetry;
        private final DcMotorEx elevatorMotor;
        private static double currentPosition;

        private final PIDController pid = new PIDController(
                TelescopeConstants.LiftConstants.KP,
                TelescopeConstants.LiftConstants.KI,
                TelescopeConstants.LiftConstants.KD
        );

        public static boolean isUsingPID = true;

        public Telescope(Telemetry telemetry, HardwareMap hardwareMap) {
            this.telemetry = telemetry;

            elevatorMotor = hardwareMap.get(DcMotorEx.class, TelescopeConstants.LiftConstants.ELEVATOR_MOTOR);
            elevatorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            elevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            pid.setTolerance(TelescopeConstants.LiftConstants.TOLERANCE);
        }

        public void update() {
            double output = pid.calculate(elevatorMotor.getCurrentPosition());
            elevatorMotor.setPower(output);
        }

        public Command setSetPoint(double setPoint) {
            return new InstantCommand(() -> pid.setSetPoint(setPoint), this);
        }

        public Command defaultCommand() {
            return new RunCommand(() -> pid.setSetPoint(TelescopeConstants.LiftConstants.DEFAULT_POSITION), this);
        }

        public static double getHeight() {
            return (currentPosition * TelescopeConstants.LiftConstants.TICK_TO_CM) + TelescopeConstants.LiftConstants.STARTING_HEIGHT;
        }

        @Override
        public void periodic() {
            if (isUsingPID){
                update();
            }

            currentPosition = elevatorMotor.getCurrentPosition();

            telemetry.addData("Lift SetPoint", pid.getSetPoint());
            telemetry.addData("Lift Position", elevatorMotor.getCurrentPosition());
            telemetry.addData("Lift Height", getHeight());
            telemetry.update();
            pid.setSetPoint(pid.getSetPoint());

        }

    }

