package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

import frc.robot.LimelightHelpers;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * =========================================================================================
 * SUBSISTEMA: LIMELIGHT (VISIÓN ARTIFICIAL Y TRACKING DE APRILTAGS)
 * =========================================================================================
 * ¿Para qué sirve?
 *   1. Detección y seguimiento de AprilTags en el campo (Speaker / Hub).
 *   2. Cálculo trigonométrico de la distancia exacta entre el robot/disparador y el objetivo.
 *   3. Transmisión de video en vivo a la Driver Station / Shuffleboard.
 *   4. Indicador visual LED para indicarle a los conductores cuando el robot está en rango de disparo.
 *
 * Hardware asociado:
 *   - Cámara inteligente Limelight conectada vía Ethernet.
 *   - 1 Salida Digital (DIO 0 en el RoboRIO) para controlar la tira de LEDs de estado.
 * =========================================================================================
 */
public class Limelight extends SubsystemBase {

    // --- PARÁMETROS GEOMÉTRICOS DE MONTAJE DE LA CÁMARA ---
    /** Altura del centro del lente de la Limelight respecto al piso (24.5 pulgadas) */
    private static final double ALTURA_LENTE_METROS = Units.inchesToMeters(24.5);
    /** Altura del centro del AprilTag en el objetivo respecto al piso (44.25 pulgadas) */
    private static final double ALTURA_OBJETIVO_METROS = Units.inchesToMeters(44.25);
    /** Ángulo de inclinación hacia arriba con el que está montada la cámara en grados */
    private static final double ANGULO_MONTAJE_GRADOS = 15.0;

    /** Índice de pipeline configurado en la interfaz web de Limelight para AprilTags */
    private static final int PIPELINE_APRILTAG = 0;

    // --- CONFIGURACIÓN DE SEÑALIZACIÓN LED (INDICADOR PARA DRIVERS) ---
    private static final int LED_PORT = 0;                  // Puerto DigitalOutput (DIO) en el roboRIO
    private static final double DISTANCIA_DISPARO_MIN = 1.0;// Distancia mínima para disparo óptimo (metros)
    private static final double DISTANCIA_DISPARO_MAX = 5.0;// Distancia máxima para disparo óptimo (metros)
    private static final double DISTANCIA_CERCA_MAX = 7.0;  // Rango de aproximación (metros)

    private final DigitalOutput m_led = new DigitalOutput(LED_PORT);

    // --- OFFSETS ESPACIALES (DESPLAZAMIENTO CÁMARA VS LANZADOR) ---
    // El lente no está exactamente en la boca del disparador, por lo que corregimos con vectores:
    // Eje X: +X es hacia adelante del robot (-4.5 in = la cámara está 4.5 in detrás del shooter).
    // Eje Y: +Y es hacia la izquierda (-9.0 in = la cámara está 9.0 in a la derecha del shooter).
    private static final double OFFSET_X_CAMARA_AL_DISPARADOR_METROS = Units.inchesToMeters(-4.5);
    private static final double OFFSET_Y_CAMARA_AL_DISPARADOR_METROS = Units.inchesToMeters(-9.0);

    /**
     * Constructor del subsistema Limelight:
     * - Selecciona el pipeline de AprilTags por defecto.
     * - Registra el stream de video MJPG en CameraServer y Shuffleboard para que los pilotos vean la cámara.
     */
    public Limelight() {
        useAprilTagPipeline();

        try {
            String[] urls = {
                    "http://limelight.local:5800/stream.mjpg",
                    "http://10.82.90.11:5800/stream.mjpg"
            };
            HttpCamera limelightFeed = new HttpCamera("Limelight", urls, HttpCameraKind.kMJPGStreamer);
            CameraServer.addCamera(limelightFeed);
            Shuffleboard.getTab("Limelight").add("Cámara", limelightFeed).withSize(4, 3);
        } catch (Exception e) {
            System.out.println("No se pudo agregar Limelight a Shuffleboard: " + e.getMessage());
        }
    }

    // =====================================================================================
    // MÉTODOS DE CONSULTA A NETWORKTABLES (VÍA LIMELIGHTHELPERS)
    // =====================================================================================

    /** @return true si la cámara está viendo algún AprilTag válido */
    public boolean tieneObjetivo() {
        return LimelightHelpers.getTV("limelight");
    }

    /** @return ID numérico del AprilTag detectado (ej. 4, 7, etc.) */
    public int getTagID() {
        return (int) LimelightHelpers.getFiducialID("limelight");
    }

    /** @return Desviación horizontal en grados (tx) respecto a la cruz central */
    public double getXOffset() {
        return LimelightHelpers.getTX("limelight");
    }

    /** @return Desviación vertical en grados (ty) respecto a la cruz central */
    public double getYOffset() {
        return LimelightHelpers.getTY("limelight");
    }

    /** Cambia el pipeline activo en la cámara */
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex("limelight", pipeline);
    }

    public void useAprilTagPipeline() {
        setPipeline(PIPELINE_APRILTAG);
    }

    // =====================================================================================
    // CÁLCULOS TRIGONOMÉTRICOS DE DISTANCIA
    // =====================================================================================

    /**
     * Calcula la distancia en línea recta sobre el suelo desde la cámara hasta el AprilTag:
     * d = (h2 - h1) / tan(a1 + a2)
     */
    public double getDistanciaCamaraMetros() {
        if (!tieneObjetivo())
            return 0.0;

        double anguloHaciaObjetivoRadianes = Math.toRadians(ANGULO_MONTAJE_GRADOS + getYOffset());
        double diferenciaAltura = ALTURA_OBJETIVO_METROS - ALTURA_LENTE_METROS;

        return diferenciaAltura / Math.tan(anguloHaciaObjetivoRadianes);
    }

    /**
     * Calcula la distancia real desde el centro del DISPARADOR (Shooter) al objetivo,
     * aplicando las correcciones de posición geométrica mediante el teorema de Pitágoras.
     */
    public double getDistanciaMetros() {
        if (!tieneObjetivo())
            return 0.0;

        double distanciaCam = getDistanciaCamaraMetros();
        double txRadianes = Math.toRadians(getXOffset());

        // Coordenadas del objetivo relativo a la cámara (+X frente, +Y izquierda)
        double targetX_Cam = distanciaCam * Math.cos(txRadianes);
        double targetY_Cam = distanciaCam * Math.sin(-txRadianes);

        // Coordenadas del objetivo pero relativas al centro del disparador
        double deltaX = targetX_Cam - OFFSET_X_CAMARA_AL_DISPARADOR_METROS;
        double deltaY = targetY_Cam - OFFSET_Y_CAMARA_AL_DISPARADOR_METROS;

        // Hipotenusa (Distancia Euclidiana real de tiro)
        return Math.hypot(deltaX, deltaY);
    }

    // =====================================================================================
    // TELEMETRÍA Y CONTROL DE LEDS (PERIODIC 20ms)
    // =====================================================================================
    @Override
    public void periodic() {
        boolean tieneObjetivo = tieneObjetivo();
        double distancia = getDistanciaMetros();

        // Enviar datos en tiempo real al Dashboard
        SmartDashboard.putBoolean("Limelight/Tiene Objetivo", tieneObjetivo);
        SmartDashboard.putNumber("Limelight/ID del AprilTag", getTagID());
        SmartDashboard.putNumber("Limelight/Angulo X", getXOffset());
        SmartDashboard.putNumber("Limelight/Angulo Y", getYOffset());

        if (tieneObjetivo) {
            SmartDashboard.putNumber("Limelight/Distancia Disparador (m)", distancia);
            SmartDashboard.putNumber("Limelight/Distancia Camara (m)", getDistanciaCamaraMetros());
            
            // --- CONTROL DE LEDS DE AVISO AL CONDUCTOR ---
            if (distancia >= DISTANCIA_DISPARO_MIN && distancia <= DISTANCIA_DISPARO_MAX) {
                // Rango óptimo de disparo: LED encendido fijo
                m_led.set(true);
                SmartDashboard.putString("Limelight/LED Status", "ESTATICO (DISPARO)");
            } else if (distancia > DISTANCIA_DISPARO_MAX && distancia <= DISTANCIA_CERCA_MAX) {
                // Acercándose al rango: LED parpadeando a 2Hz
                boolean blink = (Timer.getFPGATimestamp() % 0.5) < 0.25;
                m_led.set(blink);
                SmartDashboard.putString("Limelight/LED Status", "PARPADEO (CERCA)");
            } else {
                // Fuera de rango: LED apagado
                m_led.set(false);
                SmartDashboard.putString("Limelight/LED Status", "APAGADO (FUERA DE RANGO)");
            }
        } else {
            SmartDashboard.putNumber("Limelight/Distancia Disparador (m)", 0.0);
            SmartDashboard.putNumber("Limelight/Distancia Camara (m)", 0.0);
            m_led.set(false);
            SmartDashboard.putString("Limelight/LED Status", "APAGADO (SIN OBJETIVO)");
        }
    }
}