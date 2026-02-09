package de.egb;

import de.egb.controlcore.tests.TrajectoryAndPControllerTest;
import de.egb.core.App;

/**
 * Einstiegspunkt der SSL Zero AI Anwendung.
 */
public class Main {

    public static void main(String[] args) throws Exception {
        System.out.println("Starting SSL Zero AI");
        System.out.println("Args: " + java.util.Arrays.toString(args));

        // Spezialfall: Trajektorien-Test
        for (String s : args) {
            if ("--traj-test".equals(s)) {
                TrajectoryAndPControllerTest.run();
                return;
            }
        }

        App app = new App();
        if (!app.initialize(args)) {
            System.err.println("Initialization failed");
            System.exit(1);
        }

        Runtime.getRuntime().addShutdownHook(new Thread(app::shutdown));

        app.run();
    }
}
