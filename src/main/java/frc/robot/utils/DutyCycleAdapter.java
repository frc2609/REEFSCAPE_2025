package frc.robot.utils;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DutyCycleAdapter implements IEncoder {
    private DutyCycleEncoder encoder;

    public DutyCycleAdapter(DutyCycleEncoder encoder){
        this.encoder = encoder;
    }

    @Override
    public double getPosition(){
        return encoder.get();
    }
}
