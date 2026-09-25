package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * =========================================================================================
 * SUBSISTEMA: INTAKE (RECOLECTOR DE PIEZAS / NOTAS)
 * =========================================================================================
 * ¿Para qué sirve?
 *   Controla la recolección de piezas de juego del suelo y su alimentación hacia el Shooter.
 *   Tiene dos funciones mecánicas principales:
 *     1. Despliegue del brazo (Deploy): Sube y baja la estructura del intake mediante PID.
 *     2. Rodillos (Rollers): Dos rodillos motorizados que succionan o expulsan la pieza.
 *
 * Hardware asociado:
 *   - 1 Motor SparkMax (CAN ID 13): Articulación / Despliegue del brazo (Deploy) con Encoder.
 *   - 1 Motor SparkMax (CAN ID 18): Rodillo principal (Roller 1).
 *   - 1 Motor SparkMax (CAN ID 14): Segundo rodillo / Indexer (Roller 2).
 * =========================================================================================
 */
public class Intake extends SubsystemBase {

    // --- IDENTIFICADORES CAN EN EL BUS DEL ROBOT ---
    /** CAN ID del motor de articulación/despliegue del brazo */
    public static final int DEPLOY_MOTOR_CAN_ID = 13;
    /** CAN ID del rodillo principal de succión */
    public static final int ROLLER_MOTOR_CAN_ID = 18;
    /** CAN ID del rodillo secundario / indexer */
    public static final int ROLLER_MOTOR_2_CAN_ID = 14;

    // --- CONSTANTES DE VELOCIDAD DE DESPLIEGUE Y RODILLOS ---
    public static final double DEPLOY_BAJAR_SPEED = 0.07;           // Velocidad suave para bajar el brazo
    public static final double DEPLOY_SUBIR_SPEED = -0.22;          // Velocidad para levantar el brazo
    public static final double ROLLER_SPEED = 0.7;                  // Potencia de succión rodillo 1 (70%)
    public static final double ROLLER_2_SPEED = 0.55;               // Potencia de succión rodillo 2 (55%)
    public static final double DEPLOY_MANTENER_ABAJO_SPEED = 0.08;  // Presión hacia abajo al recolectar

    // --- PARÁMETROS DE CONTROL Y POSICIONAMIENTO ---
    private static final double DEPLOY_KP = 1.0;                    // Constante proporcional para movimiento limitado
    private static final double POSITION_TOLERANCE = 0.005;         // Tolerancia para considerar que llegó al objetivo
    private static final double FEED_SPEED = 0.3;                   // Velocidad de vaivén para alimentar el shooter
    private static final double FEED_DURATION = 0.4;                // Duración en segundos del ciclo de vaivén

    // --- POSICIONES PREDEFINIDAS DEL BRAZO (EN ROTACIONES DE ENCODER) ---
    private double[] posiciones = { 0.2, 2.4, 0.15 };
    private final String[] posicionesNombres = { "Reposo", "Abajo", "Defensa" };
    private double currentSetpoint = 0.0;
    private String activePositionName = "Reposo";

    // --- INSTANCIAS DE HARDWARE SPARK MAX ---
    private final SparkMax deployMotor = new SparkMax(DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);
    private final RelativeEncoder deployEncoder;
    private final SparkClosedLoopController deployPidController;
    private final SparkMax rollerMotor = new SparkMax(ROLLER_MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMax rollerMotor2 = new SparkMax(ROLLER_MOTOR_2_CAN_ID, MotorType.kBrushless);

    /**
     * Constructor del subsistema Intake:
     * - Configura límites suaves (Soft Limits) para evitar que el brazo choque contra el chasis.
     * - Configura el modo de freno (Brake) en el deploy y modo libre (Coast) en los rodillos.
     * - Ajusta límites de corriente (40A en deploy, 30A en rollers).
     */
    public Intake() {
        // 1. Configuración del motor del brazo (Deploy)
        SparkMaxConfig deployConfig = new SparkMaxConfig();
        deployConfig.idleMode(IdleMode.kBrake);         // Freno para mantener la posición
        deployConfig.smartCurrentLimit(40);            // Límite de 40 Amperes
        deployConfig.closedLoop
                .p(1.0)
                .i(0.0)
                .d(0.0)
                .outputRange(-0.15, 0.15);             // Limita la potencia máxima al 15% para evitar golpes

        // Límites de software de seguridad (Soft limits en rotaciones)
        deployConfig.softLimit
                .forwardSoftLimit(2.45)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(0.0)
                .reverseSoftLimitEnabled(true);

        deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        deployEncoder = deployMotor.getEncoder();
        deployEncoder.setPosition(0.0);                 // Cero inicial en posición guardada

        deployPidController = deployMotor.getClosedLoopController();

        // 2. Configuración del Rodillo 1 (Roller Principal)
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kCoast);        // Modo libre para que gire con facilidad
        rollerConfig.smartCurrentLimit(30);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 3. Configuración del Rodillo 2 (Indexer / Alimentador)
        SparkMaxConfig roller2Config = new SparkMaxConfig();
        roller2Config.idleMode(IdleMode.kCoast);
        roller2Config.smartCurrentLimit(30);
        rollerMotor2.configure(roller2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // =====================================================================================
    // MÉTODOS DEL BRAZO (DEPLOY)
    // =====================================================================================

    /** Baja el brazo con velocidad constante */
    public void bajar() {
        deployMotor.set(DEPLOY_BAJAR_SPEED);
    }

    /** Sube el brazo con velocidad constante */
    public void subir() {
        deployMotor.set(DEPLOY_SUBIR_SPEED);
    }

    /** Mueve el brazo a una posición preestablecida usando PID */
    public void irAIndice(int index) {
        if (index < 0 || index >= posiciones.length) {
            return;
        }
        activePositionName = posicionesNombres[index];
        currentSetpoint = posiciones[index];
        deployPidController.setReference(currentSetpoint, ControlType.kPosition);
    }

    public void irAHome() {
        irAIndice(0);
    }

    public void irAAbajo() {
        irAIndice(1);
    }

    public void irADefensa() {
        irAIndice(2);
    }

    public void pararDeploy() {
        deployMotor.set(0);
    }

    public void setDeploySpeed(double speed) {
        deployMotor.set(speed);
    }

    private void moverLimitado(int index, double maxOutput) {
        double error = posiciones[index] - deployEncoder.getPosition();
        double output = error * DEPLOY_KP;
        output = Math.max(-maxOutput, Math.min(maxOutput, output));
        deployMotor.set(output);
    }

    private boolean llegueAPosicion(int index) {
        return Math.abs(posiciones[index] - deployEncoder.getPosition()) < POSITION_TOLERANCE;
    }

    // =====================================================================================
    // MÉTODOS DE LOS RODILLOS (ROLLERS)
    // =====================================================================================

    /** Enciende los dos rodillos en sentido de succión/ingreso */
    public void activarRoller() {
        rollerMotor.set(ROLLER_SPEED);
        rollerMotor2.set(ROLLER_2_SPEED);
    }

    /** Invierte los dos rodillos para expulsar piezas atoradas */
    public void invertirRoller() {
        rollerMotor.set(-ROLLER_SPEED);
        rollerMotor2.set(-ROLLER_2_SPEED);
    }

    /** Detiene ambos rodillos */
    public void pararRoller() {
        rollerMotor.set(0);
        rollerMotor2.set(0);
    }

    /** Permite ajustar la velocidad de succión de forma personalizada */
    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
        rollerMotor2.set(speed != 0 ? ROLLER_2_SPEED * Math.signum(speed) : 0);
    }

    // =====================================================================================
    // MÉTODOS COMBINADOS Y DETENCIÓN TOTAL
    // =====================================================================================

    public void subirIntake() {
        subir();
    }

    /** Apaga tanto los motores de deploy como los rodillos */
    public void stop() {
        pararDeploy();
        pararRoller();
    }

    // =====================================================================================
    // FÁBRICAS DE COMANDOS (COMMANDS)
    // =====================================================================================

    public Command bajarCommand() {
        return runEnd(this::bajar, this::pararDeploy);
    }

    public Command subirManualCommand(double speed) {
        return runEnd(
                () -> {
                    deployMotor.set(DEPLOY_SUBIR_SPEED);
                    System.out.println("Intake Pos: " + deployEncoder.getPosition());
                },
                this::pararDeploy);
    }

    public Command subirCommand() {
        return runEnd(this::subirIntake, this::stop);
    }

    public Command pararRollerCommand() {
        return runOnce(this::pararRoller);
    }

    /** Activa los rodillos y aplica una ligera fuerza hacia abajo para no rebotar contra el suelo */
    public Command activarRollerCommand() {
        return runEnd(
                () -> {
                    activarRoller();
                    deployMotor.set(DEPLOY_MANTENER_ABAJO_SPEED);
                },
                () -> {
                    pararRoller();
                    deployMotor.set(0);
                });
    }

    public Command activarRollerVelocidadCommand(double speed) {
        return runEnd(() -> setRollerSpeed(speed), this::pararRoller);
    }

    public Command invertirRollerCommand() {
        return runEnd(this::invertirRoller, this::pararRoller);
    }

    public Command irAIndiceLimitadoCommand(int index, double maxOutput) {
        return runEnd(
                () -> moverLimitado(index, maxOutput),
                this::pararDeploy);
    }

    /** Secuencia para la rutina autónoma de modo defensivo */
    public Command cicloDefensaAutoCommand() {
        return Commands.waitSeconds(2.0)
                .andThen(
                        runOnce(() -> rollerMotor2.set(0.3))
                                .andThen(
                                        subirPorTiempoCommand(0.8)
                                                .andThen(bajarPorTiempoCommand(1.3)))
                                .repeatedly())
                .finallyDo((interrupted) -> {
                    stop();
                });
    }

    /** Secuencia que empuja la nota desde el intake hacia el shooter */
    public Command alimentarShooterCommand() {
        return runOnce(() -> setRollerSpeed(ROLLER_SPEED))
                .andThen(run(() -> setDeploySpeed(-FEED_SPEED)).withTimeout(FEED_DURATION))
                .andThen(run(() -> setDeploySpeed(FEED_SPEED)).withTimeout(FEED_DURATION))
                .finallyDo(this::stop);
    }

    public Command cicloDisparoCommand() {
        return runOnce(this::activarRoller)
                .andThen(bajarPorTiempoCommand(1.3))
                .andThen(Commands.waitSeconds(3.0))
                .andThen(subirPorTiempoCommand(1.0))
                .andThen(Commands.waitSeconds(0.3))
                .repeatedly()
                .finallyDo((interrupted) -> stop());
    }

    public Command grabarDatosCommand() {
        return run(() -> {
            deployMotor.set(DEPLOY_BAJAR_SPEED);
            System.out.println("Intake Posicion Actual: " + deployEncoder.getPosition());
        })
                .beforeStarting(() -> System.out.println("Intake: Buscando posición..."))
                .finallyDo((interrupted) -> {
                    pararDeploy();
                    System.out.println("Intake: Posición final: " + deployEncoder.getPosition());
                });
    }

    public Command irAHomeLentoCommand() {
        return irAIndiceLimitadoCommand(0, 0.1);
    }

    public Command irAIndiceCommand(int index) {
        return runOnce(() -> {
            irAIndice(index);
            System.out.println("Intake: Moviendo a " + activePositionName);
        });
    }

    public Command bajarPorTiempoCommand(double segundos) {
        return bajarCommand().withTimeout(segundos);
    }

    public Command subirPorTiempoCommand(double segundos) {
        return subirCommand().withTimeout(segundos);
    }

    // =====================================================================================
    // TELEMETRÍA Y PERIODIC (20ms)
    // =====================================================================================
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Posicion Actual", deployEncoder.getPosition());
        SmartDashboard.putNumber("Intake/Objetivo (Setpoint)", currentSetpoint);
        SmartDashboard.putString("Intake/Nombre Posicion", activePositionName);

        for (int i = 0; i < posiciones.length; i++) {
            String key = "Intake/Pos " + posicionesNombres[i];
            posiciones[i] = SmartDashboard.getNumber(key, posiciones[i]);
            SmartDashboard.putNumber(key, posiciones[i]);
        }
    }
}