package frc.robot.commands.pcmUtils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.PositionControlledMotor;

public class ZeroPCM extends Command {
    private final PositionControlledMotor subsystem;

    public ZeroPCM(PositionControlledMotor subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setPosition();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
