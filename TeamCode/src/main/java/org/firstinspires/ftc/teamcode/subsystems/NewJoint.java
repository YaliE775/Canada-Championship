package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NewJoint extends SubsystemBase {
    private Telemetry telemetry;

    private DcMotorEx slave;
    private DcMotorEx master;
    private DcMotorEx bothMotors;


    public NewJoint(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        Motor slave = new Motor(hardwareMap, "Angle_Slave");
        Motor master = new Motor(hardwareMap, "Angle_Master");

        MotorGroup myMotors = new MotorGroup(master, slave);

        slave.setRunMode(Motor.RunMode.PositionControl);
        master.setRunMode(Motor.RunMode.PositionControl);

        slave.setPositionCoefficient(0.05);
    }
    public Command active(){
      return new RunCommand(()-> bothMotors.setTargetPosition(1200));
    };

    
}