package frc.robot.commands.TeleopCommands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class ShootAuto extends SequentialCommandGroup {
    public ShootAuto(Drive drive) {
        addCommands(
            new Shoot(drive).withTimeout(4),
            new StopShooter(RPM.of(0), RPM.of(0), RPM.of(0)).withTimeout(0.5)
        );
    }
}
