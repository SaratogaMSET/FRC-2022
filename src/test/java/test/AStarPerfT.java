package test;

import static org.junit.Assert.*;

import org.junit.jupiter.api.Test;

import frc.robot.util.pathing.Node;
import frc.robot.util.pathing.RobotField;

public class AStarPerfT {
    private static final boolean RUN_PERFT = false;
    private static final boolean ENUMERATE_VERBOSE = false;

    @Test
    public void checkField() {
        Node node = null;
        try {
            node = new Node(0.5, 0.5, 0);
        } catch (Exception e) {
            fail("Failed to initialize Node.");
        }
        assertNotNull("Failed to initialize Node.", node);
        assertEquals("Node x is not equal to expected value of 0.5.", 0.5, node.getX(), 0);
        assertEquals("Node y is not equal to expected value of 0.5.", 0.5, node.getY(), 0);
    }

    /**
     * "Performance test, move path enumeration"
     */
    @Test
    public void perfT() {
        if (RUN_PERFT) {
            // Iterate through all nodes and verify we can generate paths
            // from the current node to all other nodes.

            // Our implementation of A* returns null if we don't find a
            // path, so we use assertNotNull to check for pathfinding failures.
            // TODO that ^

            for (int i = 0; i < RobotField.FIELD_AREA; ++i) {
                for (int j = 0; j < RobotField.FIELD_AREA; ++j) {
                    if (i != j) {
                        Node.aStar(RobotField.getNode(i), RobotField.getNode(j));
                        System.out.println("Path generated between node " + i + " and node " + j + ".");
                        if (ENUMERATE_VERBOSE) Node.printPath(RobotField.getNode(j));
                    }
                }
            }
        }
    }
}
