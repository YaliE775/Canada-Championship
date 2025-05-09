package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Manipulator extends SubsystemBase {

    private Telemetry telemetry;

    public final CRServo rightRoller;
    public final CRServo leftRoller;
    private final DistanceSensor distanceSensor;

    public Manipulator(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightRoller = new CRServo(hardwareMap , ManipulatorConstants.manipulatorConstants.RIGHT_ROLLER);
        leftRoller = new CRServo(hardwareMap , ManipulatorConstants.manipulatorConstants.LEFT_ROLLER);

        leftRoller.setInverted(true);
        rightRoller.setInverted(false);

        distanceSensor = hardwareMap.get(DistanceSensor.class, ManipulatorConstants.manipulatorConstants.DISTANCE_SENSOR);

    }

    public boolean hasSpecimen() {

        return distanceSensor.getDistance(DistanceUnit.MM) <= 38;

    }

    public boolean hasSpecimeSubmersible() {
        return distanceSensor.getDistance(DistanceUnit.MM) <= 36;
    }

    public void setRollerPower(double power) {
        leftRoller.set(power);
        rightRoller.set(power);
    }

    public Command intake() {
        return new RunCommand(() -> setRollerPower(ManipulatorConstants.manipulatorConstants.INTAKE_POWER), this);
    }

    public Command outtake() {
        return new RunCommand(() -> setRollerPower(ManipulatorConstants.manipulatorConstants.OUTTAKE_POWER), this);

    }

    public Command rest() {
        return new RunCommand(() -> setRollerPower(ManipulatorConstants.manipulatorConstants.ZERO_POWER), this);
    }

    public Command intakeUntilHasSpecimen() {
        return intake().interruptOn(this::hasSpecimen);
    }

    public Command intakeUntilHasSpecimenSubmersible() {
        return intake().interruptOn(this::hasSpecimeSubmersible);
    }

    @Override
    public void periodic() {
        telemetry.addData("the CM is", distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("has Specimen", hasSpecimen());

        telemetry.update();
    }

}