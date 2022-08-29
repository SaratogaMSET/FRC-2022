package frc.robot.util.pathing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

public class Node implements Comparable<Node> {
    private static int m_count = 0;
    private int m_id;

    private Node m_parent = null;

    private ArrayList<Edge> m_neighbors;

    private double x;
    private double y;

    public double f = Double.MAX_VALUE;
    public double g = Double.MAX_VALUE;
    public double h;

    public Node(double x, double y, double h) {
        this.h = h;
        this.x = x;
        this.y = y;
        m_id = m_count++;
        m_neighbors = new ArrayList<>();
    }

    public static class Edge {
        private double m_weight;
        private Node m_node;

        public Edge (double weight, Node node) {
            m_weight = weight;
            m_node = node;
        }

        public double getWeight() {
            return m_weight;
        }

        public Node getNode() {
            return m_node;
        }
    }

    @Override
    public int compareTo(Node n) {
        return Double.compare(this.f, n.f);
    }

    public void addBranch(double weight, Node n) {
        m_neighbors.add(new Edge(weight, n));
    }

    public double calculateHeuristic(Node n) {
        double dx = Math.abs(x - n.getX());
        double dy = Math.abs(y - n.getY());
        return 1 * (dx + dy) + (1.4 - 2 * 1) * Math.min(dx, dy);
    }

    public void setParent(Node n) {
        m_parent = n;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public int getID() {
        return m_id;
    }

    public static Node aStar(Node start, Node target) {
        PriorityQueue<Node> closed = new PriorityQueue<>();
        PriorityQueue<Node> open = new PriorityQueue<>();

        start.f = start.g + start.calculateHeuristic(target);
        open.add(start);

        while (!open.isEmpty()) {
            Node n = open.peek();
            if (n == target) {
                return n;
            }

            for (Node.Edge edge : n.m_neighbors) {
                Node m = edge.m_node;
                double totalWeight = n.g + edge.m_weight;

                if (!open.contains(m) && !closed.contains(m)) {
                    m.setParent(n);
                    m.g = totalWeight;
                    m.f = m.g + m.calculateHeuristic(target);
                    open.add(m);
                } else {
                    if (totalWeight < m.g) {
                        m.m_parent = n;
                        m.g = totalWeight;
                        m.f = m.g + m.calculateHeuristic(target);

                        if (closed.contains(m)) {
                            closed.remove(m);
                            open.add(m);
                        }
                    }
                }
            }

            open.remove(n);
            closed.add(n);
        }

        return null;
    }

    public static void printPath(Node target) {
        Node n = target;

        if (n == null) return;

        List<Integer> ids= new ArrayList<>();

        while (n.m_parent != null) {
            ids.add(n.m_id);
            n = n.m_parent;
        }
        ids.add(n.m_id);
        Collections.reverse(ids);

        for (int id : ids) {
            System.out.print(id + " ");
        }
        System.out.println();
    }
}
