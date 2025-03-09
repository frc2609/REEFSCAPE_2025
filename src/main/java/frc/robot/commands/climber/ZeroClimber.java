package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;


public class ZeroClimber extends Command {
    private final Climber climber;

    public 
    ZeroClimber(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.goToPosition(-50);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop climber when command ends
        climber.stop();
    }
}
