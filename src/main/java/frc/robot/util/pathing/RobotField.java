package frc.robot.util.pathing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;

public class RobotField {
    // Field dimensions in feet
    public static final int FIELD_LENGTH = 54; // 54 - 10 for testing
    public static final int FIELD_WIDTH = 27; // 27 - 5 for testing
    public static final int FIELD_AREA = FIELD_LENGTH * FIELD_WIDTH;

    // Weight values
    private static final double CARDINAL_WEIGHT = 1d; // value assigned to N/S/E/W connections
    private static final double DIAGONAL_WEIGHT = 1.4d; // value assigned to diagonal connections

    private static RobotField m_instance = null;

    private static final FieldNode[] m_nodes = initNodes();
    private static final double[][] m_weights = initWeights();

    private static final boolean DEBUG = true;
    private static final boolean DEBUG_VERBOSE = true;

    private enum NodeType {
        CORNER,
        EDGE,
        INNER
    }

    private RobotField() {}

    private static final Node[] initNodes(boolean b) {
        System.out.println("Initializing nodes...");
        double now = System.currentTimeMillis();
        
        Node[] res = new Node[FIELD_AREA];

        for (int i = 0; i < FIELD_AREA; ++i) {
            res[i] = new Node(0.5 + (i % FIELD_LENGTH), 0.5 + (int)(i / FIELD_LENGTH), 0);
        }

        System.out.println("Nodes initialized in " + (System.currentTimeMillis() - now) + " ms.");

        return res;
    }

    private static final FieldNode[] initNodes() {
        System.out.println("Initializing nodes...");
        double now = System.currentTimeMillis();

        FieldNode[] res = new FieldNode[FIELD_AREA];

        for (int i = 0; i < FIELD_AREA; ++i) {
            res[i] = new FieldNode(0.5 + (i % FIELD_LENGTH), 0.5 + (int)(i / FIELD_LENGTH));
        }

        System.out.println("Nodes initialized in " + (System.currentTimeMillis() - now) + " ms.");

        return res;
    }

    private static final double[][] initWeights() {
        double now = System.currentTimeMillis();

        double[][] res = new double[FIELD_AREA][FIELD_AREA];

        // Zero all weights
        int i, j, c;
        for (i = 0; i < FIELD_AREA; ++i) {
            for (j = 0; j < FIELD_AREA; ++j) {
                res[i][j] = 0.0;
            }
        }

        // Initialize weights
        // TODO simplify and optimize logic
        NodeType type = null;
        FieldNode currNode = null;
        double currX, currY;
        for (i = 0; i < FIELD_AREA; ++i) {
            currNode = m_nodes[i];
            currX = currNode.m_x - 0.5;
            currY = currNode.m_y - 0.5;

            if ( ( (currX % (FIELD_LENGTH - 1) ) + (currY % (FIELD_WIDTH - 1) ) ) == 0) {
                type = NodeType.CORNER;
                if (DEBUG_VERBOSE) {
                    System.out.println("Node " + i + " is type CORNER and has adjusted coordinates (" + currX + ", " + currY + ").");
                }
            } else if ( ( (currX % (FIELD_LENGTH - 1)) * (currY % (FIELD_WIDTH - 1)) ) == 0) {
                type = NodeType.EDGE;
                if (DEBUG_VERBOSE) {
                    System.out.println("Node " + i + " is type EDGE and has adjusted coordinates (" + currX + ", " + currY + ").");
                }
            } else if (
                !( ( (currX % (FIELD_LENGTH - 1) ) + (currY % (FIELD_WIDTH - 1) ) ) == 0) &&
                !( ( (currX % (FIELD_LENGTH - 1)) * (currY % (FIELD_WIDTH - 1)) ) == 0)
            ) {
                type = NodeType.INNER;
                if (DEBUG_VERBOSE) {
                    System.out.println("Node " + i + " is type INNER and has adjusted coordinates (" + currX + ", " + currY + ").");
                }
            }

            if (type == NodeType.CORNER) {
                c = 0;

                if (i == 0) {
                    for (j = 0; j < FIELD_AREA; ++j) {
                        // TODO remove if/else
                        /* 
                         * res[i][i + FIELD_LENGTH] = CARDINAL_WEIGHT;
                         * res[i][i + 1] = CARDINAL_WEIGHT;
                         * res[i][i + 1 + FIELD_LENGTH] = CARDINAL_WEIGHT;
                         * etc.
                         */
                        if (j == i + FIELD_LENGTH) {
                            res[i][j] = CARDINAL_WEIGHT;
                            ++c;
                        } else if (j == i + 1) {
                            res[i][j] = CARDINAL_WEIGHT;
                            ++c;
                        } else if (j == i + 1 + FIELD_LENGTH) {
                            res[i][j] = DIAGONAL_WEIGHT;
                            ++c;
                        }
                    }

                    if (DEBUG) {
                        assertEquals(3, c, "Number of connections for node " + i + " should be 3, but is " + c + ".");
                    }
                } else if (i == FIELD_LENGTH - 1) {
                    for (j = 0; j < FIELD_AREA; ++j) {
                        if (j == i - 1) {
                            res[i][j] = CARDINAL_WEIGHT;
                            ++c;
                        } else if (j == i + FIELD_LENGTH) {
                            res[i][j] = CARDINAL_WEIGHT;
                            ++c;
                        } else if (j == i + FIELD_LENGTH - 1) {
                            res[i][j] = DIAGONAL_WEIGHT;
                            ++c;
                        }
                    }

                    if (DEBUG) {
                        assertEquals(3, c, "Number of connections for node " + i + " should be 3, but is " + c + ".");
                    }
                } else if (i == FIELD_LENGTH * (FIELD_WIDTH - 1)) {
                    for (j = 0; j < FIELD_AREA; ++j) {
                        if (j == i - FIELD_LENGTH) {
                            res[i][j] = CARDINAL_WEIGHT;
                            ++c;
                        } else if (j == i - FIELD_LENGTH + 1) {
                            res[i][j] = DIAGONAL_WEIGHT;
                            ++c;
                        } else if (j == i + 1) {
                            res[i][j] = CARDINAL_WEIGHT;
                            ++c;
                        }
                    }

                    if (DEBUG) {
                        assertEquals(3, c, "Number of connections for node " + i + " should be 3, but is " + c + ".");
                    }
                } else if (i == (FIELD_LENGTH * FIELD_WIDTH) - 1) {
                    for (j = 0; j < FIELD_AREA; ++j) {
                        if (j == i - 1) {
                            res[i][j] = CARDINAL_WEIGHT;
                            ++c;
                        } else if (j == i - FIELD_LENGTH) {
                            res[i][j] = CARDINAL_WEIGHT;
                            ++c;
                        } else if (j == i - FIELD_LENGTH - 1) {
                            res[i][j] = DIAGONAL_WEIGHT;
                            ++c;
                        }
                    }

                    if (DEBUG) {
                        assertEquals(3, c, "Number of connections for node " + i + " should be 3, but is " + c + ".");
                    }
                }
            } else if (type == NodeType.EDGE) {
                c = 0;

                if (currX == 0) {
                    for (j = 0; j < FIELD_AREA; ++j) {
                        if (j == i - FIELD_LENGTH || j == i + 1 || j == i + FIELD_LENGTH) {
                            res[i][j] = CARDINAL_WEIGHT;
                            ++c;
                        } else if (j == i + FIELD_LENGTH + 1 || j == i - FIELD_LENGTH + 1) {
                            res[i][j] = DIAGONAL_WEIGHT;
                            ++c;
                        }
                    }

                    if (DEBUG) {
                        assertEquals(5, c, "Number of connections for node " + i + " should be 5, but is " + c + ".");
                    }
                } else if (currX == FIELD_LENGTH - 1) {
                    for (j = 0; j < FIELD_AREA; ++j) {
                        if (j == i - FIELD_LENGTH || j == i - 1 || j == i + FIELD_LENGTH) {
                            res[i][j] = CARDINAL_WEIGHT;
                            ++c;
                        } else if (j == i + FIELD_LENGTH - 1 || j == i - FIELD_LENGTH - 1) {
                            res[i][j] = DIAGONAL_WEIGHT;
                            ++c;
                        }
                    }

                    if (DEBUG) {
                        assertEquals(5, c, "Number of connections for node " + i + " should be 5, but is " + c + ".");
                    }
                } else if (currY == 0) {
                    for (j = 0; j < FIELD_AREA; ++j) {
                        if (j == i - 1 || j == i + 1 || j == i + FIELD_LENGTH) {
                            res[i][j] = CARDINAL_WEIGHT;
                            ++c;
                        } else if (j == i - 1 + FIELD_LENGTH || j == i + 1 + FIELD_LENGTH) {
                            res[i][j] = DIAGONAL_WEIGHT;
                            ++c;
                        }
                    }

                    if (DEBUG) {
                        assertEquals(5, c, "Number of connections for node " + i + " should be 5, but is " + c + ".");
                    }
                } else if (currY == FIELD_WIDTH - 1) {
                    for (j = 0; j < FIELD_AREA; ++j) {
                        if (j == i + 1 || j == i - 1 || j == i - FIELD_LENGTH) {
                            res[i][j] = CARDINAL_WEIGHT;
                            ++c;
                        } else if (j == i + 1 - FIELD_LENGTH || j == i - 1 - FIELD_LENGTH) {
                            res[i][j] = DIAGONAL_WEIGHT;
                            ++c;
                        }
                    }

                    if (DEBUG) {
                        assertEquals(5, c, "Number of connections for node " + i + " should be 5, but is " + c + ".");
                    }
                }
            } else if (type == NodeType.INNER) {
                c = 0;

                for (j = 0; j < FIELD_AREA; ++j) {
                    if (j == i - 1 || j == i + 1 || j == i + FIELD_LENGTH || j == i - FIELD_LENGTH) {
                        res[i][j] = CARDINAL_WEIGHT;
                        ++c;
                    } else if (
                        j == i - 1 + FIELD_LENGTH || j == i + 1 + FIELD_LENGTH || 
                        j == i - 1 - FIELD_LENGTH || j == i + 1 - FIELD_LENGTH
                    ) {
                        res[i][j] = DIAGONAL_WEIGHT;
                        ++c;
                    }
                }

                if (DEBUG) {
                    assertEquals(8, c, "Number of connections for node " + i + " should be 8, but is " + c + ".");
                }
            }
        }
        
        System.out.println("Weights initialized in " + (System.currentTimeMillis() - now) + " ms.");

        return res;
    }

    public FieldNode getNode(int n) {
        return m_nodes[n];
    }

    public double[] getWeights(int n) {
        return m_weights[n];
    }

    /**
     * Returns whether or not two nodes are connected.
     * 
     * @param n The source node
     * @param m The destination node
     * @return Whether there exists a connection between the two nodes
     */
    public boolean isConnected(int n, int m) {
        return m_weights[n][m] != 0;
    }

    /**
     * Returns size of our field representation for debugging purposes
     * 
     * @return Size of m_nodes
     */
    public int getFieldSize() {
        return m_nodes.length;
    }

    /**
     * Returns size of our node weight array for debugging purposes
     * 
     * @return Size of m_weights
     */
    public int getWeightSize() {
        return m_weights.length;
    }

    /**
     * Returns size of a given node's weight array for debugging purposes
     * 
     * @param n Index of the weight array
     * @return The size of node n's weight array
     */
    public int getSubWeightSize(int n) {
        return m_weights[n].length;
    }

    public FieldNode[] getNodes() {
        return m_nodes;
    }

    public static ArrayList<FieldNode> getNodesAsArrayList() {
        ArrayList<FieldNode> res = new ArrayList<>();
        
        for (int i = 0; i < FIELD_AREA; ++i) {
            res.add(m_nodes[i]);
        }

        return res;
    }

    public static RobotField getInstance() {
        if (m_instance == null) {
            m_instance = new RobotField();
        }

        return m_instance;
    }
}
