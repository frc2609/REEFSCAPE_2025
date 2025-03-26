package frc.robot.commands.reefStuff.L4Coral;

import frc.robot.subsystems.Led;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class L4LED extends Command {

  private final Led led;
  /**
   * Creates a new RainbowLed.
   *
   * @param L4LED The subsystem used by this command.
   * @param coral
   */
  public L4LED(Led L4LED) {
    this.led = L4LED;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(L4LED);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void execute() {
    led.elevatorlvl4();
    led.Coral();
  }

  public boolean isFinished() {
    return false;
  }
}