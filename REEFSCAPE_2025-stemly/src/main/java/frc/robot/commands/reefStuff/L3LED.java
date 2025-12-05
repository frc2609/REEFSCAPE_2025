package frc.robot.commands.reefStuff;

import frc.robot.subsystems.Led;
import edu.wpi.first.wpilibj2.command.Command;

public class L3LED extends Command{
    private final Led led;

    /**
     * Creates a new RainbowLed.
     *
     * @param RedLed The subsystem used by this command.
     */
    public L3LED (Led L3LED) {
        this.led = L3LED;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(L3LED);
    }
  
    @Override
    public void execute() {
        led.elevatorlvl3();
        led.Coral();
    }
  
    public boolean isFinished() {
        return false;
    }
}
