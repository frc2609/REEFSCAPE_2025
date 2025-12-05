package frc.robot.commands.reefStuff;

import frc.robot.subsystems.Led;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class L2LED extends Command {

    private final Led led;

    /**
     * Creates a new RainbowLed.
     *
     * @param RedLed The subsystem used by this command.
     */
    public L2LED(Led L2LED) {
        this.led = L2LED;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements( L2LED);
    }

    @Override
    public void execute() {
        led.elevatorlvl2();
        led.Coral();
    }

    public boolean isFinished() {
        return false;
    }
}