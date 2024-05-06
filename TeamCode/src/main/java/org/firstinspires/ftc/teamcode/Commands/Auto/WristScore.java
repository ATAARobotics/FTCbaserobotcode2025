package org.firstinspires.ftc.teamcode.Commands.Auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class WristScore extends CommandBase {

    // The subsystem the command runs on
    private final WristSubsystem m_wrist;

    public WristScore(WristSubsystem mWrist) {
        m_wrist = mWrist;
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        m_wrist.setWristPosition(Wrist.DEPOSIT_LONG);
    }

    @Override
    public boolean isFinished() {
        return m_wrist.isInPosition();
    }

}