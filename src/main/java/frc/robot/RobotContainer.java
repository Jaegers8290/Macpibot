package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Gancho;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * =========================================================================================
 * ROBOT CONTAINER: NÚCLEO DE ASIGNACIÓN DE MANDOS, COMANDOS Y AUTÓNOMOS
 * =========================================================================================
 * ¿Para qué sirve?
 *   Aquí se instancian todos los subsistemas del robot, se declaran los controles de los pilotos
 *   y se configuran los disparadores (triggers/botones) y las secuencias de PathPlanner.
 *
 * Dispositivos de control (OI - Operator Interface):
 *   - Puerto 0: Mando de PlayStation 5 (Piloto principal - Chasis Swerve, apuntado y velocidad).
 *   - Puerto 1: Mando de Xbox (Copiloto / Operador - Intake, Gancho y mecanismos).
 *   - Puerto 2: Joystick 3D USB (Piloto secundario / Joystick de soporte).
 * =========================================================================================
 */
public class RobotContainer {

    // --- LÍMITES DINÁMICOS DEL SWERVE ---
    /** Velocidad máxima lineal en m/s calculada a 12V desde TunerConstants */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    
    /** 
     * Velocidad de rotación máxima (0.45 rotaciones/segundo = ~2.82 rad/s).
     * Reducida de 0.75 a 0.45 para proteger los engranajes de dirección de esfuerzos bruscos.
     */
    private double MaxAngularRate = RotationsPerSecond.of(0.45).in(RadiansPerSecond);

    /** 
     * Multiplicador dinámico de velocidad: permite al piloto regular la velocidad del robot
     * entre 20% y 100% sobre la marcha usando la cruceta (D-Pad) del mando.
     */
    private double speedMultiplier = 1.0;

    // --- SOLICITUDES DE CONTROL PHOENIX 6 (SWERVE REQUESTS) ---
    /** Modo Field-Centric: el robot se mueve relativo a la perspectiva del campo (adelante siempre es hacia el fondo) */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)                      // Zona muerta del 10% en traslación
            .withRotationalDeadband(MaxAngularRate * 0.1)      // Zona muerta del 10% en giro
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /** Modo Freno: orienta las ruedas en forma de X para evitar ser empujado */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    /** Modo Apuntar: orienta las 4 ruedas hacia un ángulo específico */
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // --- TELEMETRÍA Y CONTROLADORES ---
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandPS5Controller joystick = new CommandPS5Controller(0); // Piloto principal (PS5)
    private final CommandXboxController operator = new CommandXboxController(1); // Operador de mecanismos (Xbox)
    private final Joystick joystick2 = new Joystick(2);                          // Joystick 3D auxiliar

    // --- INSTANCIAS GLOBALES DE LOS SUBSISTEMAS ---
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Limelight limelight = new Limelight();
    public final Gancho gancho = new Gancho();

    // --- SELECTOR DE AUTÓNOMOS EN SMARTDASHBOARD ---
    private final SendableChooser<Command> autoChooser;

    /**
     * Constructor principal del contenedor del robot:
     * Registra comandos nombrados para PathPlanner, inicializa AutoBuilder y vincula botones.
     */
    public RobotContainer() {
        registerNamedCommands();
        drivetrain.configureAutoBuilder();

        // Carga la lista desplegable de trayectorias autónomas en SmartDashboard
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    /**
     * Registra comandos con nombre para que la aplicación gráfica PathPlanner
     * pueda ejecutarlos en cualquier punto de una trayectoria autónoma.
     */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("BajarIntake", intake.bajarPorTiempoCommand(1.3));
        NamedCommands.registerCommand("Subir", intake.subirPorTiempoCommand(1.0));
        NamedCommands.registerCommand("PararRoller", intake.pararRollerCommand());
        NamedCommands.registerCommand("Roller", intake.activarRollerCommand().withTimeout(4.0));
        NamedCommands.registerCommand("GanchoReposo", gancho.irAIndiceCommand(0));
        NamedCommands.registerCommand("GanchoSubir", gancho.subirPorTiempoSafeCommand());
        NamedCommands.registerCommand("GanchoBajar", gancho.bajarPorTiempoSafeCommand());
        NamedCommands.registerCommand("CicloDefensaAuto", intake.cicloDefensaAutoCommand());
    }

    /**
     * Mapeo de botones físicos de los mandos a acciones y comandos del robot.
     */
    private void configureBindings() {

        // =================================================================================
        // 1. CONTROL PRINCIPAL DEL SWERVE (MANDO PS5 + JOYSTICK 3D)
        // =================================================================================
        // Se mezcla la entrada del stick izquierdo de PS5 y del Joystick 3D con zona muerta
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                .withVelocityX((MathUtil.applyDeadband(-joystick.getLeftY(), 0.1)
                        + MathUtil.applyDeadband(-joystick2.getY(), 0.1))
                        * MaxSpeed * speedMultiplier)
                .withVelocityY((MathUtil.applyDeadband(-joystick.getLeftX(), 0.1)
                        + MathUtil.applyDeadband(-joystick2.getX(), 0.1))
                        * MaxSpeed * speedMultiplier)
                .withRotationalRate((MathUtil.applyDeadband(-joystick.getRightX(), 0.1)
                        + MathUtil.applyDeadband(-joystick2.getTwist(), 0.1))
                        * MaxAngularRate * speedMultiplier)));

        // Modo inactivo cuando el robot está deshabilitado
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Botón 10 (PS5 Touchpad / Options): Resetear el ángulo hacia adelante del campo
        joystick.button(10).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
        // Botón Cruz (X): Bloquear ruedas en X (freno anti-empuje)
        joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        
        // Botón Triángulo: Alinear ruedas rectas (0°)
        joystick.triangle().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(Rotation2d.kZero)));
        
        // Botón Círculo: Apuntar ruedas hacia la dirección indicada por el stick
        joystick.circle().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // =================================================================================
        // 2. AJUSTE DE VELOCIDAD DINÁMICA CON D-PAD (PS5)
        // =================================================================================
        // Flecha izquierda: Subir velocidad en incrementos de 10% (hasta 100%)
        joystick.povLeft().onTrue(Commands.runOnce(() -> {
            speedMultiplier = Math.min(1.0, speedMultiplier + 0.1);
            SmartDashboard.putNumber("Swerve Speed %", speedMultiplier * 100);
        }));
        // Flecha derecha: Reducir velocidad en incrementos de 10% (mínimo 20%)
        joystick.povRight().onTrue(Commands.runOnce(() -> {
            speedMultiplier = Math.max(0.2, speedMultiplier - 0.1);
            SmartDashboard.putNumber("Swerve Speed %", speedMultiplier * 100);
        }));

        // =================================================================================
        // 3. AUTO-ALIGN CON APRILTAG (LIMELIGHT) + DISPARO AUTOMÁTICO
        // =================================================================================
        // Botón R1 (Botón 6): El robot conduce manualmente pero gira automáticamente hacia el AprilTag
        // y a la vez modula la velocidad del disparador según la distancia leída por la cámara
        joystick.button(6).whileTrue(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(MathUtil.applyDeadband(joystick.getLeftY(), 0.2)
                                * MaxSpeed * speedMultiplier)
                        .withVelocityY(MathUtil.applyDeadband(-joystick.getLeftX(), 0.2)
                                * MaxSpeed * speedMultiplier)
                        .withRotationalRate(limelight.tieneObjetivo()
                                ? -limelight.getXOffset() * 0.05 * MaxAngularRate
                                : 0))
                        .alongWith(shooter.dispararSegunDistanciaCommand(
                                limelight::getDistanciaMetros)));

        // Botón R2 (Botón 8): Disparo a velocidad fija
        joystick.button(8).whileTrue(shooter.shootCommand());

        // Botón L2 (Botón 7): Disparo adaptativo según distancia sin auto-alinear chasis
        joystick.button(7).whileTrue(shooter.dispararSegunDistanciaCommand(limelight::getDistanciaMetros));
        
        // Gatillo Joystick 3D (Botón 1): Disparo fijo de respaldo
        new JoystickButton(joystick2, 1).whileTrue(shooter.shootCommand());

        // Registro de telemetría de odometría y señal
        drivetrain.registerTelemetry(logger::telemeterize);

        // =================================================================================
        // 4. CONTROL DE MECANISMOS (MANDO XBOX OPERADOR)
        // =================================================================================
        // Botón B: Bajar Intake por tiempo (0.7s)
        operator.b().onTrue(intake.bajarPorTiempoCommand(0.7));
        // Botón A: Subir Intake por tiempo (0.7s)
        operator.a().onTrue(intake.subirPorTiempoCommand(0.7));
        // Botón X: Activar rodillos de succión (mientras se mantenga presionado)
        operator.x().whileTrue(intake.activarRollerCommand());
        // Botón Y: Invertir rodillos para expulsar (mientras se mantenga presionado)
        operator.y().whileTrue(intake.invertirRollerCommand());

        // Bumpers (LB / RB): Control manual del gancho de escalado (80% potencia)
        operator.leftBumper().whileTrue(gancho.subirManualCommand(0.8));
        operator.rightBumper().whileTrue(gancho.bajarManualCommand(0.8));

        // D-Pad Arriba / Abajo: Rutina segura de subida/bajada del gancho
        operator.povUp().onTrue(gancho.subirPorTiempoSafeCommand());
        operator.povDown().onTrue(gancho.bajarPorTiempoSafeCommand());
    }

    /** Retorna la rutina autónoma seleccionada por los conductores en el Dashboard */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
