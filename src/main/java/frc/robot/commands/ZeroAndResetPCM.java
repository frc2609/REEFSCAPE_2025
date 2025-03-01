package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.PositionControlledMotor;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ZeroAndResetPCM extends Command {
    private final PositionControlledMotor subsystem;
    private final ProfiledPIDController pidController;
    private final double targetPosition;  // Typically 0

    public ZeroAndResetPCM(PositionControlledMotor subsystem) {
        this.subsystem = subsystem;
        this.pidController = subsystem.pidController;
        this.targetPosition = subsystem.zeroPosition;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // Reset the PID controller's state using the current absolute position.
        pidController.reset(subsystem.getAbsPosition());
        pidController.setGoal(targetPosition);
        pidController.setTolerance(subsystem.positionTolerance);
        pidController.enableContinuousInput(0, 1);
    }

    @Override
    public void execute() {
        // Get the current encoder reading (assumed to be the absolute position)
        double currentPosition = subsystem.getAbsPosition();
        SmartDashboard.putNumber("command pos", currentPosition);
        // Calculate the PID output based on the error
        double output = pidController.calculate(currentPosition);
        // Command the motor with the computed output voltage
        subsystem.setVoltage(output);
    }

    @Override
    public boolean isFinished() {
        // The command finishes when the PID controller determines we're at the target
        return pidController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor and reset its internal position to zero
        subsystem.stop();
        subsystem.resetPosition(); // This method should call motor.setPosition(0) (and do the same for any follower motors)   
    }

}
