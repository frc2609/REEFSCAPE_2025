package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ZeroAndResetClimber extends Command {
    private final Climber climber;
    private final ProfiledPIDController pidController;
    private final double targetPosition;  // Typically 0

    public ZeroAndResetClimber(Climber climber) {
        this.climber = climber;
        this.pidController = climber.pidController;
        this.targetPosition = climber.zeroPosition;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        // Reset the PID controller's state using the current absolute position.
        pidController.reset(climber.getAbsPosition());
        pidController.setGoal(targetPosition);
        pidController.setTolerance(climber.positionTolerance);
    }

    @Override
    public void execute() {
        // Get the current encoder reading (assumed to be the absolute position)
        double currentPosition = climber.getAbsPosition();
        SmartDashboard.putNumber("command pos", currentPosition);
        // Calculate the PID output based on the error
        double output = pidController.calculate(currentPosition);
        // Command the motor with the computed output voltage
        climber.setVoltage(output);
    }

    @Override
    public boolean isFinished() {
        // The command finishes when the PID controller determines we're at the target
        return pidController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor and reset its internal position to zero
        climber.stop();
        climber.resetPosition(); // This method should call motor.setPosition(0) (and do the same for any follower motors)
    }
}
