package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * =========================================================================================
 * TELEMETRÍA DEL CHASIS SWERVE (CTRE PHOENIX 6)
 * =========================================================================================
 * ¿Para qué sirve?
 *   Captura en tiempo real cada actualización de odometría del tren motriz Swerve (Pose 2D,
 *   velocidades de chasis, estados de módulos y fallos de adquisición de datos CAN)
 *   y las envía a los archivos de registro (Hoot Logs) de CTRE mediante SignalLogger.
 *
 * ¿De dónde viene?
 *   Es llamado automáticamente en cada ciclo por el SwerveDrivetrain a través del callback
 *   registrado en RobotContainer (`drivetrain.registerTelemetry(logger::telemeterize)`).
 * =========================================================================================
 */
public class Telemetry {
    private final double MaxSpeed;

    /**
     * @param maxSpeed Velocidad máxima configurada del chasis en m/s
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start(); // Inicia la grabación continua de telemetría de alta frecuencia
    }

    /**
     * Registra el estado completo del Swerve en cada iteración del hilo de odometría (250Hz).
     * @param state Objeto con los datos de posición, velocidad y módulos del Swerve
     */
    public void telemeterize(SwerveDriveState state) {
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
        SignalLogger.writeInteger("DriveState/FailedDaqs", state.FailedDaqs);
    }
}
