package frc.robot.commands.shooter;

import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Shooter;

public class InsideShot extends SetShooterSpeed {
    
    public InsideShot(Shooter shooter) {
        super(shooter, Shooters.INSIDE_SPEED, Shooters.INSIDE_BSPEED);
    }
}
