import java.util.*;
import java.util.stream.Collector;
import java.util.stream.Collectors;

/**
 * Created by martinpettersson on 07/01/17.
 */
public class Pathfinder {
    private HashMap<Integer, Node> inputMap;

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

        // Build hash map from input vector
        inputMap = new HashMap<Integer, Node>();
        for (int yCoordinate = 0; yCoordinate < nMapHeight; yCoordinate++) {
            for (int xCoordinate = 0; xCoordinate < nMapWidth; xCoordinate++) {
                inputMap.put(
                        yCoordinate * nMapWidth + xCoordinate,
                        new Node(xCoordinate, yCoordinate));
            }
        }

        System.err.println("---- test equals");
        Node n1 = new Node(0,0);
        Node n2 = new Node(0,0);
        System.err.println(n1.equals(n2));
        System.err.println("---- test equals");
        Node startNode = new Node(nStartX, nStartY);

        HashSet<Node> closedSet = new HashSet<>();
        HashSet<Node> openSet = new HashSet<>();
        openSet.add(startNode);
        HashMap<Node, Node> cameFrom = new HashMap<>();
        startNode.setgScore(0);
        Node goalNode = new Node(nTargetX, nTargetY);
        startNode.setfScore(startNode.heuristicDistanceFunction(goalNode));

        while (!openSet.isEmpty()) {

            Node currentNode = Collections.min(openSet);
            System.err.println(currentNode.getX() + " " + currentNode.getY());
            if (currentNode.isEqual(goalNode)) {
                ArrayList<Node> totalPath = reconstructPath(cameFrom, currentNode);
                outputMapAndPath(pMap, totalPath, nMapWidth, nMapHeight);
                return totalPath.size();
            }

            openSet.remove(currentNode);
            closedSet.add(currentNode);
            ArrayList<Node> neighborsOfCurrent = getNeighbors(currentNode, pMap, nMapWidth);

            for (Node neighbor : neighborsOfCurrent) {
                if (closedSet.contains(neighbor)) continue;
                int tentativeGScore = currentNode.getgScore()
                        + currentNode.heuristicDistanceFunction(neighbor); // dist_between?
                if (!openSet.contains(neighbor)) openSet.add(neighbor);
                else if (tentativeGScore >= neighbor.getgScore())  continue;
                cameFrom.put(neighbor, currentNode);
                neighbor.setgScore(tentativeGScore);
                neighbor.setfScore(neighbor.getgScore()
                        + neighbor.heuristicDistanceFunction(goalNode));
            }
        }

        return 0;
    }

    private ArrayList<Node> getNeighbors(Node currentNode, int[] pMap, int width) {
        ArrayList<Node> neighbors = new ArrayList<>();
        int mapSize = pMap.length;
        boolean isNextToLeftWall = currentNode.getX() == 0;
        boolean isNextToRightWall = currentNode.getX() == width - 1;
        int currentNodeIndex = currentNode.getX() + currentNode.getY() * width;
        if ((currentNodeIndex + 1) >= 0 && (currentNodeIndex + 1) < mapSize && !isNextToRightWall)
            neighbors.add(inputMap.get(currentNodeIndex + 1));
        if ((currentNodeIndex - 1) >= 0 && (currentNodeIndex - 1) < mapSize && !isNextToLeftWall)
            neighbors.add(inputMap.get(currentNodeIndex - 1));
        if ((currentNodeIndex + width) >= 0 && (currentNodeIndex + width) < mapSize)
            neighbors.add(inputMap.get(currentNodeIndex + width));
        if ((currentNodeIndex - width) >= 0 && (currentNodeIndex - width) < mapSize)
            neighbors.add(inputMap.get(currentNodeIndex - width));
        return neighbors;
    }

    private ArrayList<Node> reconstructPath(HashMap<Node, Node> cameFrom, Node currentNode) {
        ArrayList<Node> totalPath = new ArrayList<>();
        totalPath.add(currentNode);
        while (cameFrom.keySet().contains(currentNode)) {
            currentNode = cameFrom.get(currentNode);
            totalPath.add(currentNode);
        }

        totalPath.stream().forEach(
                n -> System.err.println(n.getX() + " " + n.getY() + ", " + n.inputIndex(4)));
        System.err.println(totalPath.size());
        return totalPath;
    }

    private void outputMapAndPath(int[] pMap, ArrayList<Node> totalPath, int nMapWidth, int nMapHeight) {

        List<Integer> pathIndices = totalPath
                .stream()
                .map(n -> n.inputIndex(nMapWidth))
                .collect(Collectors.toList());
        System.err.println("");

        for (int yCoordinate = nMapHeight - 1; yCoordinate >= 0; yCoordinate--) {
            for (int xCoordinate = 0; xCoordinate < nMapWidth; xCoordinate++) {
                if (pathIndices.contains(yCoordinate * nMapWidth + xCoordinate))
                    System.err.print("x");
                else System.err.print("-");
            }
            System.err.println("");
        }
    }
}

class Node implements Comparable<Node> {
    private int xPosition;
    private int yPosition;
    private int fScore;
    private int gScore;

    public Node(int xPosition, int yPosition) {
        this.xPosition = xPosition;
        this.yPosition = yPosition;
    }

    public int heuristicDistanceFunction(Node destinationNode) {
        int xDistance = destinationNode.getX() - xPosition;
        int yDistance = destinationNode.getY() - yPosition;
        return Math.abs(xDistance) + Math.abs(yDistance);
    }

    public boolean isEqual(Node otherNode) {
        if (this.getX() == otherNode.getX() && this.getY() == otherNode.getY())
            return true;
        else return false;
    }

    @Override
    public int compareTo(Node o) {
        return this.getfScore() - o.getfScore();
    }

    public int inputIndex(int width) {
        return yPosition * width + xPosition;
    }

    public void setgScore(int gScore) { this.gScore = gScore; }

    public void setfScore(int fScore) { this.fScore = fScore; }

    public int getfScore() { return fScore; }

    public int getgScore() { return gScore; }

    public int getX() { return xPosition; }

    public int getY() { return yPosition; }
}
