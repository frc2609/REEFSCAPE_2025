package frc.robot.commands.climber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class StopClimberCommand extends Command{
    Climber climber;
    public StopClimberCommand(Climber climber){
        this.climber = climber;
        addRequirements(climber);
    }
    @Override
    public void initialize() {
        climber.disablePositionControl();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
