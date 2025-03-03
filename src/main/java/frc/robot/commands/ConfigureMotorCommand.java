package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.PositionControlledMotor;

public class ConfigureMotorCommand extends Command {
    private final PositionControlledMotor motor;

    public ConfigureMotorCommand(PositionControlledMotor subsystem) {
        this.motor = subsystem;
    }

    @Override
    public void initialize() {
        // motor.updateTalonConfig();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
} 