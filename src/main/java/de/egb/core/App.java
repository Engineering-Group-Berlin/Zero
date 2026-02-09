package de.egb.core;

import de.egb.controlcore.*;
import de.egb.controlcore.control.Se2PController;
import de.egb.debugstream.DebugSnapshot;
import de.egb.debugstream.JSONFrameWriter;
import de.egb.vision.VisionReceiver;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

/**
 * Zentrale Anwendungsklasse für SSL Zero AI.
 * Kümmert sich um Initialisierung, Hauptschleife und Shutdown.
 */
public class App {

    private volatile boolean running = false;
    private int stdDeltaTimeMs = 100;
    private int actualDeltaTimeMs = stdDeltaTimeMs;

    // Kernkomponenten
    private WorldStateProvider worldStateProvider;
    private VisionReceiver visionReceiver;
    private Thread visionThread;

    // Debug-Export
    private boolean doDebugExport;
    private JSONFrameWriter debugWriter;
    private long startTimeNanos;

    // Controller (für Debug-Export)
    private Se2PController debugController;
    private int tick;

    public int getStdDeltaTimeMs() {
        return stdDeltaTimeMs;
    }

    public void setStdDeltaTimeMs(int ms) {
        this.stdDeltaTimeMs = ms;
        this.actualDeltaTimeMs = ms;
    }

    public int getActualDeltaTimeMs() {
        return actualDeltaTimeMs;
    }

    public void setActualDeltaTimeMs(int ms) {
        this.actualDeltaTimeMs = ms;
    }

    /**
     * Initialisiert die App mit den gegebenen Argumenten.
     * Muss vor {@link #run()} aufgerufen werden.
     */
    public boolean initialize(String[] args) {
        doDebugExport = parseDebugExport(args);

        worldStateProvider = new WorldStateProvider();
        visionReceiver = new VisionReceiver(worldStateProvider);
        visionThread = new Thread(visionReceiver);

        if (doDebugExport) {
            Path debugPath = Path.of("build/debug/recording.jsonl");
            System.out.println("Debug export enabled: " + debugPath);
            try {
                debugWriter = new JSONFrameWriter(debugPath, false);
            } catch (IOException e) {
                System.err.println("Debug export init failed: " + e.getMessage());
                doDebugExport = false;
            }
            if (debugWriter != null) {
                startTimeNanos = System.nanoTime();
                JSONFrameWriter w = debugWriter;
                Runtime.getRuntime().addShutdownHook(new Thread(() -> {
                    try {
                        w.flush();
                        w.close();
                    } catch (IOException ignored) {}
                }));
                Limits limits = Limits.defaults();
                debugController = new Se2PController(2.0, 4.0, limits);
            }
        }

        return true;
    }

    private static boolean parseDebugExport(String[] args) {
        for (String s : args) {
            if ("--debug-export".equals(s)) {
                System.out.println("arg debug export found");
                System.out.flush();
                return true;
            }
        }
        return false;
    }

    public void run() {
        running = true;
        visionThread.start();
        System.out.println("SSL Zero AI started");

        mainLoop();
    }

    private void mainLoop() {
        double dt = actualDeltaTimeMs / 1000.0;
        Pose2D targetPose = new Pose2D(new Vec2(1.0, 0.5), 0.0);
        Twist2D targetTwist = new Twist2D(Vec2.ZERO, 0.0);

        while (running) {
            try {
                Thread.sleep(actualDeltaTimeMs);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
            tick++;

            if (debugWriter != null) {
                performDebugExport(dt, targetPose, targetTwist);
            }
        }
    }

    private void performDebugExport(double dt, Pose2D targetPose, Twist2D targetTwist) {
        double t = (System.nanoTime() - startTimeNanos) * 1e-9;
        WorldState ws = worldStateProvider.getCurrent();

        var ctrlList = new ArrayList<DebugSnapshot.CtrlData>();
        if (debugController != null) {
            for (int i = 0; i < WorldState.MAX_ROBOTS_PER_TEAM; i++) {
                var rsBlue = ws.blue.robots[i];
                Pose2D poseBlue = rsBlue.state.pose();
                Se2PController.Command cmdBlue = debugController.update(poseBlue, targetPose, targetTwist, dt);
                ctrlList.add(new DebugSnapshot.CtrlData("blue", i, targetPose, targetTwist, new Twist2D(cmdBlue.v(), cmdBlue.omega())));
            }
            for (int i = 0; i < WorldState.MAX_ROBOTS_PER_TEAM; i++) {
                var rsYellow = ws.yellow.robots[i];
                Pose2D poseYellow = rsYellow.state.pose();
                Se2PController.Command cmdYellow = debugController.update(poseYellow, targetPose, targetTwist, dt);
                ctrlList.add(new DebugSnapshot.CtrlData("yellow", i, targetPose, targetTwist, new Twist2D(cmdYellow.v(), cmdYellow.omega())));
            }
        }

        DebugSnapshot snapshot = DebugSnapshot.from(ws, t, ctrlList);
        try {
            debugWriter.write(snapshot);
            if (tick % 10 == 0) {
                debugWriter.flush();
            }
        } catch (IOException e) {
            System.err.println("Debug export write failed: " + e.getMessage());
        }
    }

    public void shutdown() {
        running = false;
        if (visionThread != null && visionThread.isAlive()) {
            visionThread.interrupt();
            try {
                visionThread.join(2000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        if (debugWriter != null) {
            try {
                debugWriter.flush();
                debugWriter.close();
            } catch (IOException ignored) {}
        }
        System.out.println("Shutting down application");
        System.out.flush();
    }
}
