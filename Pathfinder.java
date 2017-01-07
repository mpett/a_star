import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;

/**
 * Created by martinpettersson on 10/10/16.
 */
public class Pathfinder {
    public static void main(String[] args) {
        new Pathfinder();
    }

    public Pathfinder() {
        int[] pMap = {1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1};
        int[] pOutBuffer = new int[12];
        findPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);
    }

    private int findPath(int nStartX, int nStartY, int nTargetX, int nTargetY,
                         int[] pMap, int nMapWidth, int nMapHeight, int[] pOutBuffer, int nOutBufferSize) {
        ArrayList<Node> nodes = new ArrayList<Node>();
        nodes.add(new Node(0,1,3,5));
        nodes.add(new Node(0,1,3,43));
        nodes.add(new Node(0,1,3,7));
        nodes.add(new Node(0,1,3,8));
        nodes.add(new Node(0,1,3,44));
        nodes.add(new Node(0,1,3,42));
        nodes.add(new Node(0,1,3,125));

        // Build hash map of input vector
        HashMap<Integer, Node> inputMap = new HashMap<Integer, Node>();
        for (int yCoordinate = 0; yCoordinate < nMapHeight; yCoordinate++) {
            for (int xCoordinate = 0; xCoordinate < nMapWidth; xCoordinate++) {
                inputMap.put(yCoordinate * nMapWidth + xCoordinate, new Node(xCoordinate, yCoordinate, 0, 0));
            }
        }

        Node lolNode = new Node(0,1,0,0);
        ArrayList<Integer> neighbors = getNeighbors(lolNode, pMap, nMapWidth);
        System.err.println("---");
        for (int n : neighbors) System.err.println(n);
        System.err.println("---");

        for (Node node : nodes) System.err.println(node.getCurrentPriority());
        Collections.sort(nodes);
        System.err.println("---");
        for (Node node : nodes) System.err.println(node.getCurrentPriority());
        ArrayList<Node> closedSet = new ArrayList<Node>();
        ArrayList<Node> openSet = new ArrayList<Node>();
        HashMap<Node, Node> cameFrom = new HashMap<Node, Node>();
        Node startNode = new Node(nStartX, nStartY, 0, 0);
        startNode.updatePriority(nTargetX, nTargetY);
        openSet.add(startNode);

        while (!openSet.isEmpty()) {
            Node currentNode = Collections.min(openSet);
            if (currentNode.getxPosition() == nTargetX &&
                    currentNode.getyPosition() == nTargetY)
                return reconstructPath(cameFrom, currentNode);
            openSet.remove(currentNode);
            closedSet.add(currentNode);
            ArrayList<Integer> neighborIndices = getNeighbors(currentNode, pMap, nMapWidth);
            // Construct list of neighbor nodes
            ArrayList<Node> neighborNodes = new ArrayList<Node>();
            System.err.println("----- Neigbor indices");
            for (int neighborIndex : neighborIndices) {
                System.err.println(neighborIndex);
                if (pMap[neighborIndex] != 0) {
                    neighborNodes.add(inputMap.get(neighborIndex));
                }
            }
            System.err.println("---- Neighbor nodes");
            for (Node neighbor : neighborNodes) {
                System.err.println("I ended up here: " + neighborNodes.size());
                System.err.println(neighbor.getxPosition() + " " + neighbor.getyPosition());
                if (closedSet.contains(neighbor)) {
                    System.err.println("closed set contained neighbor");
                    continue;
                }
                int tentativeGScore = neighbor.tentativePriority(currentNode.getxPosition(), currentNode.getyPosition());
                if (!openSet.contains(neighbor)) {
                    openSet.add(neighbor);
                    System.err.println("added neighbor to open set");
                }
                if (tentativeGScore >= neighbor.getCurrentPriority()) {
                    System.err.println("was larger than current priority");
                    continue;
                }

                cameFrom.put(neighbor, currentNode);
                neighbor.updatePriority(currentNode.getxPosition(), currentNode.getyPosition());
                System.err.println("I put neighbor in came from");
            }

        }
        return 0;
    }

    private int reconstructPath(HashMap<Node, Node> cameFrom, Node currentNode) {
        System.err.println("I called reconstruct path.");
        ArrayList<Node> totalPath = new ArrayList<Node>();
        totalPath.add(currentNode);
        System.err.println(cameFrom.size());
        while (cameFrom.containsKey(currentNode)) {
            currentNode = cameFrom.get(currentNode);
            totalPath.add(currentNode);
        }

        System.err.println("--- total path");
        for (Node n : totalPath)
            System.err.println(n.getxPosition() + " " + n.getyPosition());
        return 0;
    }

    private ArrayList<Integer> getNeighbors(Node currentNode, int[] pMap, int width) {
        ArrayList<Integer> neighbors = new ArrayList<Integer>();
        int mapSize = pMap.length;
        boolean isNextToLeftWall = currentNode.getxPosition() == 0;
        boolean isNextToRightWall = currentNode.getxPosition() == width - 1;
        int currentNodeIndex = currentNode.getxPosition() + currentNode.getyPosition() * width;
        if ((currentNodeIndex + 1) >= 0 && (currentNodeIndex + 1) < mapSize && !isNextToRightWall)
            neighbors.add(currentNodeIndex + 1);
        if ((currentNodeIndex - 1) >= 0 && (currentNodeIndex - 1) < mapSize && !isNextToLeftWall)
            neighbors.add(currentNodeIndex - 1);
        if ((currentNodeIndex + width) >= 0 && (currentNodeIndex + width) < mapSize)
            neighbors.add(currentNodeIndex + width);
        if ((currentNodeIndex - width) >= 0 && (currentNodeIndex - width) < mapSize)
            neighbors.add(currentNodeIndex - width);
        return neighbors;
    }

    public class Node implements Comparator<Node>, Comparable<Node> {
        int xPosition;
        int yPosition;
        int totalDistance;
        int currentPriority;

        public Node(int xPosition, int yPosition, int distance, int priority) {
            this.xPosition = xPosition;
            this.yPosition = yPosition;
            totalDistance = distance;
            currentPriority = priority;
        }

        public int tentativePriority(int xDestination, int yDestination) {
         return totalDistance + heuristicDistanceFunction(xDestination, yDestination) * 10;

        }

        public void updatePriority(int xDestination, int yDestination) {
            currentPriority = totalDistance + heuristicDistanceFunction(xDestination, yDestination) * 10;
        }

        private int heuristicDistanceFunction(int xDestination, int yDestination) {
            int xDistance = xDestination - xPosition;
            int yDistance = yDestination - yPosition;
            return Math.abs(xDistance) + Math.abs(yDistance);
        }

        public int getxPosition() {
            return xPosition;
        }

        public int getyPosition() {
            return yPosition;
        }

        public int getTotalDistance() {
            return totalDistance;
        }

        public int getCurrentPriority() {
            return currentPriority;
        }

        @Override
        public int compare(Node o1, Node o2) {
            return o1.getCurrentPriority() - o2.getCurrentPriority();
        }

        @Override
        public int compareTo(Node o) {
            return this.getCurrentPriority() - o.getCurrentPriority();
        }
    }
}


