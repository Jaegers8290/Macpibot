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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * =========================================================================================
 * SUBSISTEMA: GANCHO (CLIMBER / ELEVADOR DE ESCALADO)
 * =========================================================================================
 * ¿Para qué sirve?
 *   Controla el mecanismo de escalado/colgado del robot en la estructura del campo.
 *   Permite desplegar los brazos del gancho y elevar el chasis del robot para ganar puntos.
 *
 * Hardware asociado:
 *   - 1 Motor NEO / NEO 550 controlado por un controlador SPARK MAX (CAN ID 15).
 *   - 1 Encoder integrado en el motor para leer rotaciones y posición relativa.
 *   - Controlador PID interno en el SparkMax para posicionamiento preciso.
 * =========================================================================================
 */
public class Gancho extends SubsystemBase {

    // --- IDENTIFICADORES Y VELOCIDADES (CONSTANTES) ---
    /** ID del controlador SparkMax en la red CAN del robot */
    public static final int MOTOR_CAN_ID = 15;

    /** Velocidades manuales y automáticas (valores entre -1.0 y 1.0) */
    public static final double SUBIR_SPEED = 0.5;
    public static final double BAJAR_SPEED = -0.5;
    public static final double SUPIR_AUTO = -0.6;
    public static final double BAJAR_AUTO = 0.7;
    public static final double MANUAL_SPEED_50 = 1.0;
    public static final double GRABAR_SPEED = -0.4;

    /** 
     * Factores de conversión de unidades del encoder:
     * 1.0 / 75.0 reduce las rotaciones del motor considerando la caja reductora 75:1.
     */
    public static final double CONVERSION_FACTOR_POSITION = 1.0 / 75.0;
    public static final double CONVERSION_FACTOR_VELOCITY = CONVERSION_FACTOR_POSITION / 60.0;

    // --- GANANCIAS DEL LAZO CERRADO (PID) ---
    private static final double kP = 1.0; // Ganancia proporcional
    private static final double kI = 0.0; // Ganancia integral
    private static final double kD = 0.0; // Ganancia derivativa

    // --- POSICIONES PREDEFINIDAS DEL GANCHO ---
    /** Posiciones del encoder para cada estado del gancho */
    private double[] posiciones = { 0.0, 200.00, 0.0 };
    private final String[] posicionesNombres = { "Reposo", "Subir", "Bajar" };

    // --- OBJETOS DE HARDWARE REV ROBOTICS ---
    /** Instancia del motor SparkMax conectado al CAN ID 15 */
    private final SparkMax motor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
    /** Lector del encoder interno del motor */
    private final RelativeEncoder encoder;
    /** Controlador PID de lazo cerrado integrado en el firmware del SparkMax */
    private final SparkClosedLoopController pidController;

    // --- VARIABLES DE ESTADO Y DIAGNÓSTICO ---
    private double currentSetpoint = 0.0;
    private String activePositionName = "Unknown";
    private boolean isUp = false;       // Bandera para saber si el gancho está arriba o abajo
    private int tickCounter = 0;        // Contador de ciclos para latido visual (Heartbeat)

    /**
     * Constructor del subsistema Gancho:
     * Configura el modo de freno (Brake), límite de corriente (40A), factores de conversión
     * y ganancias PID en la memoria no volátil del SparkMax.
     */
    public Gancho() {
        System.out.println("Gancho: Inicializado Subsystem Constructor");
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Freno activo cuando no recibe potencia (para sostener el peso del robot colgado)
        config.idleMode(IdleMode.kBrake);
        
        // Límite de corriente inteligente para evitar quemar el motor bajo esfuerzo
        config.smartCurrentLimit(40);
        
        // Configuración de ganancias PID y límites de salida de voltaje
        config.closedLoop
                .p(kP)
                .i(kI)
                .d(kD)
                .outputRange(-0.5, 0.5);
                
        // Configuración de escalas de posición y velocidad del encoder
        config.encoder
                .positionConversionFactor(CONVERSION_FACTOR_POSITION)
                .velocityConversionFactor(CONVERSION_FACTOR_VELOCITY);

        // Aplica y persiste los parámetros en el SparkMax
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        encoder = motor.getEncoder();
        encoder.setPosition(0.0); // Calibrar posición inicial a 0
        pidController = motor.getClosedLoopController();
    }

    // =====================================================================================
    // MÉTODOS DE CONTROL POR POSICIÓN (PID)
    // =====================================================================================

    /**
     * Envía al gancho a un índice específico del arreglo de posiciones.
     * @param index 0=Reposo, 1=Subir, 2=Bajar
     */
    public void irAIndice(int index) {
        if (index < 0 || index >= posiciones.length)
            return;
        activePositionName = posicionesNombres[index];
        irAPosicion(posiciones[index]);
    }

    public void irAReposo() {
        irAIndice(0);
    }

    public void irASubir() {
        irAIndice(1);
    }

    public void irABajar() {
        irAIndice(2);
    }

    /**
     * Aplica el setpoint de posición directamente al controlador PID interno.
     */
    public void irAPosicion(double posicion) {
        currentSetpoint = posicion;
        pidController.setReference(posicion, ControlType.kPosition);
    }

    // =====================================================================================
    // MÉTODOS DE CONTROL MANUAL POR VOLTAJE/POTENCIA
    // =====================================================================================

    /** Sube el gancho con la velocidad manual fija */
    public void subir() {
        motor.set(SUBIR_SPEED);
    }

    /** Baja el gancho con la velocidad manual fija */
    public void bajar() {
        motor.set(BAJAR_SPEED);
    }

    /** Detiene el motor por completo */
    public void stop() {
        motor.set(0);
    }

    /** Obtiene la posición actual leída por el encoder */
    public double getPosicion() {
        return encoder.getPosition();
    }

    // =====================================================================================
    // FÁBRICAS DE COMANDOS (COMMANDS) PARA BOTONES Y AUTÓNOMO
    // =====================================================================================

    public Command Abajo() {
        return runOnce(() -> irAPosicion(0.0));
    }

    public Command Arriba() {
        return runOnce(() -> irAPosicion(-20));
    }

    /** Mueve a un índice de posición predefinido */
    public Command irAIndiceCommand(int index) {
        return runOnce(() -> {
            irAIndice(index);
            System.out.println("Gancho: Moviendo a " + activePositionName);
        });
    }

    public Command irAReposoCommand() {
        return runOnce(this::irAReposo).onlyIf(() -> currentSetpoint != posiciones[0]);
    }

    public Command irASubirCommand() {
        return runOnce(this::irASubir).onlyIf(() -> currentSetpoint != posiciones[1]);
    }

    public Command irABajarCommand() {
        return runOnce(this::irABajar).onlyIf(() -> currentSetpoint != posiciones[2]);
    }

    /** Comando continuo que sube mientras el botón esté presionado y frena al soltar */
    public Command subirCommand() {
        return runEnd(this::subir, this::stop);
    }

    /** Comando continuo que baja mientras el botón esté presionado y frena al soltar */
    public Command bajarCommand() {
        return runEnd(this::bajar, this::stop);
    }

    /** Mueve manualmente a una velocidad dada y frena al terminar */
    public Command moverManualCommand(double speed) {
        return runEnd(
                () -> {
                    motor.set(speed);
                    System.out.println("Gancho Pos: " + getPosicion());
                },
                this::stop);
    }

    public Command subirManualCommand(double speed) {
        return moverManualCommand(speed);
    }

    public Command bajarManualCommand(double speed) {
        return moverManualCommand(-speed);
    }

    /** Mueve el gancho por una cantidad fija de segundos */
    public Command moverPorTiempoCommand(double speed, double segundos) {
        return runEnd(() -> motor.set(speed), this::stop).withTimeout(segundos);
    }

    /** Sube de forma segura por tiempo asegurando que no se ejecute dos veces seguidas */
    public Command subirPorTiempoSafeCommand() {
        return moverPorTiempoCommand(SUPIR_AUTO, 5.0)
                .onlyIf(() -> !isUp)
                .finallyDo((interrupted) -> {
                    if (!interrupted) {
                        isUp = true;
                    }
                });
    }

    /** Baja de forma segura por tiempo asegurando que solo baje si previamente subió */
    public Command bajarPorTiempoSafeCommand() {
        return moverPorTiempoCommand(BAJAR_AUTO, 5.0)
                .onlyIf(() -> isUp)
                .finallyDo((interrupted) -> {
                    if (!interrupted) {
                        isUp = false;
                    }
                });
    }

    public Command subir90Por5SegundosCommand() {
        return moverPorTiempoCommand(0.9, 5.0);
    }

    public Command bajar40Por6SegundosCommand() {
        return moverPorTiempoCommand(-0.4, 6.0);
    }

    public Command mueveteManual50Command(boolean reverse) {
        double speed = reverse ? -MANUAL_SPEED_50 : MANUAL_SPEED_50;
        return runEnd(() -> motor.set(speed), this::stop);
    }

    public Command grabarDatosCommand() {
        return run(() -> {
            motor.set(GRABAR_SPEED);
            System.out.println("Gancho Posicion Actual: " + getPosicion());
        })
                .beforeStarting(() -> System.out.println("Gancho: Buscando posición..."))
                .finallyDo((interrupted) -> {
                    stop();
                    System.out.println("Gancho: Posición final detenida: " + getPosicion());
                });
    }

    // =====================================================================================
    // MÉTODO PERIODIC (EJECUTADO CADA 20ms EN EL ROBOT)
    // =====================================================================================
    @Override
    public void periodic() {
        tickCounter++;
        // Latido visual en SmartDashboard para confirmar comunicación
        SmartDashboard.putBoolean("Gancho/Vivo (Heartbeat)", (tickCounter / 50) % 2 == 0);

        // Envío de telemetría y datos de calibración al SmartDashboard / Shuffleboard
        double currentPos = getPosicion();
        SmartDashboard.putNumber("Gancho/Posicion Actual", currentPos);
        SmartDashboard.putNumber("Gancho/Calibrar - Copia este valor", currentPos);
        SmartDashboard.putNumber("Gancho/Objetivo (Setpoint)", currentSetpoint);
        SmartDashboard.putString("Gancho/Nombre Posicion", activePositionName);

        // Permite ajustar las posiciones en vivo desde la interfaz de SmartDashboard
        for (int i = 0; i < posiciones.length; i++) {
            String key = "Gancho/Pos " + posicionesNombres[i];
            posiciones[i] = SmartDashboard.getNumber(key, posiciones[i]);
            SmartDashboard.putNumber(key, posiciones[i]);
        }
    }
}