package frc.robot.util.pathing;

public class RobotField {
    // Field dimensions in feet
    public static final int FIELD_LENGTH = 54; // 54 - 10 for testing
    public static final int FIELD_WIDTH = 27; // 27 - 5 for testing
    public static final int FIELD_AREA = FIELD_LENGTH * FIELD_WIDTH;

    // Weight values
    public static final double CARDINAL_WEIGHT = 1d; // value assigned to N/S/E/W connections
    public static final double DIAGONAL_WEIGHT = 1.4d; // value assigned to diagonal connections

    private static final Node[] m_nodes = initNodes();

    private static final boolean DEBUG = true;

    private enum NodeType {
        CORNER,
        EDGE,
        INNER
    }

    private RobotField() {

    }

    private static final Node[] initNodes() {
        System.out.println("Initializing nodes...");
        double now = System.nanoTime();
        
        Node[] res = new Node[FIELD_AREA];

        for (int i = 0; i < FIELD_AREA; ++i) {
            res[i] = new Node(0.5 + (i % FIELD_LENGTH), 0.5 + (int)(i / FIELD_LENGTH), 0);
        }

        // Initialize weights
        NodeType type = null;
        Node currNode = null;
        double currX = 0, currY = 0;
        for (int i = 0; i < FIELD_AREA; ++i) {
            currNode = res[i];
            currX = currNode.getX() - 0.5;
            currY = currNode.getY() - 0.5;

            if ( ( (currX % (FIELD_LENGTH - 1) ) + (currY % (FIELD_WIDTH - 1) ) ) == 0) {
                type = NodeType.CORNER;
                if (DEBUG) {
                    System.out.println("Node " + i + " is type CORNER and has adjusted coordinates (" + currX + ", " + currY + ").");
                }
            } else if ( ( (currX % (FIELD_LENGTH - 1)) * (currY % (FIELD_WIDTH - 1)) ) == 0) {
                type = NodeType.EDGE;
                if (DEBUG) {
                    System.out.println("Node " + i + " is type EDGE and has adjusted coordinates (" + currX + ", " + currY + ").");
                }
            } else if (
                !( ( (currX % (FIELD_LENGTH - 1) ) + (currY % (FIELD_WIDTH - 1) ) ) == 0) &&
                !( ( (currX % (FIELD_LENGTH - 1)) * (currY % (FIELD_WIDTH - 1)) ) == 0)
            ) {
                type = NodeType.INNER;
                if (DEBUG) {
                    System.out.println("Node " + i + " is type INNER and has adjusted coordinates (" + currX + ", " + currY + ").");
                }
            }

            if (type == NodeType.CORNER) {
                if (i == 0) {
                    res[i].addBranch(CARDINAL_WEIGHT, res[i + FIELD_LENGTH]);
                    res[i].addBranch(CARDINAL_WEIGHT, res[i + 1]);
                    res[i].addBranch(DIAGONAL_WEIGHT, res[i + 1 + FIELD_LENGTH]);
                } else if (i == FIELD_LENGTH - 1) {
                    res[i].addBranch(CARDINAL_WEIGHT, res[i - 1]);
                    res[i].addBranch(CARDINAL_WEIGHT, res[i + FIELD_LENGTH]);
                    res[i].addBranch(DIAGONAL_WEIGHT, res[i - 1 + FIELD_LENGTH]);
                } else if (i == FIELD_LENGTH * (FIELD_WIDTH - 1)) {
                    res[i].addBranch(CARDINAL_WEIGHT, res[i - FIELD_LENGTH]);
                    res[i].addBranch(CARDINAL_WEIGHT, res[i - FIELD_LENGTH + 1]);
                    res[i].addBranch(DIAGONAL_WEIGHT, res[i + 1]);
                } else if (i == (FIELD_LENGTH * FIELD_WIDTH) - 1) {
                    res[i].addBranch(CARDINAL_WEIGHT, res[i - 1]);
                    res[i].addBranch(CARDINAL_WEIGHT, res[i - FIELD_LENGTH]);
                    res[i].addBranch(DIAGONAL_WEIGHT, res[i - FIELD_LENGTH - 1]);
                }
            } else if (type == NodeType.EDGE) {
                if (currX == 0) {
                    res[i].addBranch(CARDINAL_WEIGHT, res[i - FIELD_LENGTH]);
                    res[i].addBranch(CARDINAL_WEIGHT, res[i + 1]);
                    res[i].addBranch(CARDINAL_WEIGHT, res[i + FIELD_LENGTH]);
                    res[i].addBranch(DIAGONAL_WEIGHT, res[i + FIELD_LENGTH + 1]);
                    res[i].addBranch(DIAGONAL_WEIGHT, res[i - FIELD_LENGTH + 1]);
                } else if (currX == FIELD_LENGTH - 1) {
                    res[i].addBranch(CARDINAL_WEIGHT, res[i - FIELD_LENGTH]);
                    res[i].addBranch(CARDINAL_WEIGHT, res[i - 1]);
                    res[i].addBranch(CARDINAL_WEIGHT, res[i + FIELD_LENGTH]);
                    res[i].addBranch(DIAGONAL_WEIGHT, res[i + FIELD_LENGTH - 1]);
                    res[i].addBranch(DIAGONAL_WEIGHT, res[i - FIELD_LENGTH - 1]);
                } else if (currY == 0) {
                    res[i].addBranch(CARDINAL_WEIGHT, res[i - 1]);
                    res[i].addBranch(CARDINAL_WEIGHT, res[i + 1]);
                    res[i].addBranch(CARDINAL_WEIGHT, res[i + FIELD_LENGTH]);
                    res[i].addBranch(DIAGONAL_WEIGHT, res[i - 1 + FIELD_LENGTH]);
                    res[i].addBranch(DIAGONAL_WEIGHT, res[i + 1 + FIELD_LENGTH]);
                } else if (currY == FIELD_WIDTH - 1) {
                    res[i].addBranch(CARDINAL_WEIGHT, res[i + 1]);
                    res[i].addBranch(CARDINAL_WEIGHT, res[i - 1]);
                    res[i].addBranch(CARDINAL_WEIGHT, res[i - FIELD_LENGTH]);
                    res[i].addBranch(DIAGONAL_WEIGHT, res[i + 1 - FIELD_LENGTH]);
                    res[i].addBranch(DIAGONAL_WEIGHT, res[i - 1 - FIELD_LENGTH]);
                }
            } else if (type == NodeType.INNER) {
                res[i].addBranch(CARDINAL_WEIGHT, res[i - 1]);
                res[i].addBranch(CARDINAL_WEIGHT, res[i + 1]);
                res[i].addBranch(CARDINAL_WEIGHT, res[i + FIELD_LENGTH]);
                res[i].addBranch(CARDINAL_WEIGHT, res[i - FIELD_LENGTH]);
                res[i].addBranch(DIAGONAL_WEIGHT, res[i - 1 + FIELD_LENGTH]);
                res[i].addBranch(DIAGONAL_WEIGHT, res[i + 1 + FIELD_LENGTH]);
                res[i].addBranch(DIAGONAL_WEIGHT, res[i - 1 - FIELD_LENGTH]);
                res[i].addBranch(DIAGONAL_WEIGHT, res[i + 1 - FIELD_LENGTH]);
            }
        }

        System.out.println("Nodes initialized in " + (System.nanoTime() - now) + " ms.");

        return res;
    }

    public static Node getNode(int n) {
        return m_nodes[n];
    }
}
