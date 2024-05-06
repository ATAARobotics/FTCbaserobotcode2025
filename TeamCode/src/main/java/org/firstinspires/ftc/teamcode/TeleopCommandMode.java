package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Teleop Commands",group = "")
public class TeleopCommandMode extends CommandOpMode {

    @Override
    public void initialize() {
        ATAARobot m_robot = new ATAARobot(Constants.OpModeType.TELEOP, hardwareMap, new ATAATelemetry(telemetry));
    }

}
