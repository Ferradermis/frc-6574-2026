package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class MapleSimIntake {
    private final IntakeSimulation intakeSimulation;

    public MapleSimIntake(SwerveDriveSimulation swerveDriveSimulation) {

        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            "Fuel", 
            swerveDriveSimulation, 
            Meters.of(0.635),
            Meters.of(0.2667),
            IntakeSimulation.IntakeSide.BACK,
            70);
    }

    public void setRunning(boolean runIntake) {
        if (runIntake)
            intakeSimulation.startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game pieces
        else
            intakeSimulation.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
    }

    public boolean isFuelInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
    }
}
