package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Teleop extends CommandOpMode {
    private GamepadEx driverController;
    private DcMotorEx motor;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "1");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driverController = new GamepadEx(gamepad1);

        driverController.getGamepadButton(GamepadKeys.Button.B).whileActiveContinuous(new RunCommand(() -> motor.setPower(0.5)));
        driverController.getGamepadButton(GamepadKeys.Button.X).whileActiveContinuous(new RunCommand(() -> motor.setPower(-0.5)));
        driverController.getGamepadButton(GamepadKeys.Button.Y).whileActiveContinuous(new RunCommand(() -> motor.setPower(0)));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Encder Value: ", motor.getCurrentPosition());
        telemetry.addData("Power: ", motor.getPower());
        telemetry.update();
    }

}
