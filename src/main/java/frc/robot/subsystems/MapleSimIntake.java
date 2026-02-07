package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.drive.Drive;

public class MapleSimIntake {
    private final IntakeSimulation intakeSimulation;
    private final AbstractDriveTrainSimulation driveTrainSimulation;
    private final DriveTrainSimulationConfig driveTrainSimulationConfig;
    private final GyroSimulation gyroSimulation;
    private final SwerveModuleSimulation swerveModuleSimulation;
    private final SwerveModuleSimulationConfig swerveModuleSimulationConfig;

    public MapleSimIntake(Drive drive) {

        gyroSimulation = new GyroSimulation(0.001, 0.05);
        swerveModuleSimulationConfig = new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60Foc(1), 
            DCMotor.getKrakenX60Foc(1), 
            5.357142857142857, 
            21.428571428571427, Volts.of(0.2), 
            Volts.of(0.2), 
            Inches.of(1.536), 
            KilogramSquareMeters.of(0.004), 0);

        swerveModuleSimulation = new SwerveModuleSimulation(swerveModuleSimulationConfig);

        // driveTrainSimulationConfig = new DriveTrainSimulationConfig(
        // Pounds.of(100), 
        // Inches.of(27.5), 
        // Inches.of(27.5), 
        // Inches.of(2), 
        // Inches.of(2), 
        // gyroSimulation, 
        // swerveModuleSimulation);

        driveTrainSimulation = new AbstractDriveTrainSimulation(null, null) {

            @Override
            public void simulationSubTick() {

            }
            
        };

        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            "Fuel", 
            driveTrainSimulation, 
            Meters.of(1),
            Meters.of(1),
            IntakeSimulation.IntakeSide.BACK,
            0);
    }
}
