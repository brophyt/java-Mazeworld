package mazeworld;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;

import java.util.Comparator;



public abstract class UUSearchProblem {
	
	protected int nodesExplored;
	protected int maxMemory;
	protected UUSearchNode startNode;
	
	protected interface UUSearchNode {
		public ArrayList<UUSearchNode> getSuccessors();
		public ArrayList<UUSearchNode> getSuccessorsWithHeuristic();
		public boolean goalTest();
		public boolean individualGoalTest();
		public int getDepth();
		public int getx();
		public int gety();
	}
	
	public List<UUSearchNode> breadthFirstSearch() {
		resetStats();
		ArrayList<UUSearchNode> returnList;
		HashMap<UUSearchNode, UUSearchNode> visited = new HashMap<UUSearchNode, UUSearchNode>();
		LinkedList<UUSearchNode> frontier = new LinkedList<UUSearchNode>();
		
		frontier.add(startNode);
		visited.put(startNode, null);
		
		while (frontier.size() > 0) {
			UUSearchNode current = frontier.removeFirst();
			
			if (current.goalTest()) {
				returnList = backchain(current, visited);
				return returnList;	
			}
			
			ArrayList<UUSearchNode> successors = current.getSuccessors();
			for (int i = 0; i < successors.size(); i++) {
				UUSearchNode successor = successors.get(i);
				if (!visited.containsKey(successor)) {
					frontier.add(successor);
					visited.put(successor, current);
				}
			}
		}
		return null;
	}
	
	public List<UUSearchNode> AStarSearch() {
		System.out.println("\nFinding a path using AStar...");
		resetStats();
		ArrayList<UUSearchNode> returnList;
		HashSet<UUSearchNode> visited = new HashSet<UUSearchNode>();
		HashMap<UUSearchNode, UUSearchNode> backtracker = new HashMap<UUSearchNode, UUSearchNode>();
		HashMap<UUSearchNode, Integer> distanceMap = new HashMap<UUSearchNode, Integer>();
		Comparator<UUSearchNode> comparator = new AStarComparator();
		PriorityQueue<UUSearchNode> frontier = new PriorityQueue<UUSearchNode>(comparator);	
		
		frontier.add(startNode);
		
		while (frontier.size() > 0) {
			UUSearchNode current = frontier.poll();
			if (current.goalTest()) {
				System.out.println("Path Found!");
				returnList = backchain(current, backtracker);
				return returnList;	
			}
			
			visited.add(current);
			ArrayList<UUSearchNode> successors = current.getSuccessorsWithHeuristic();
			if (successors != null) {
				for (int i = 0; i < successors.size(); i++) {
					UUSearchNode successor = successors.get(i);
					if (!visited.contains(successor) && !frontier.contains(successor)) {
						frontier.add(successor);
						backtracker.put(successor, current);
						distanceMap.put(successor, successor.getDepth());
					} else if (frontier.contains(successor)) {
						if (distanceMap.get(successor) != null && 
								successor.getDepth() < distanceMap.get(successor)) {
							frontier.add(successor);
							backtracker.put(successor, current);
							distanceMap.put(successor, successor.getDepth());
						}
					}
				}
			}
		}
		return null;	
	}
	
	
	// backchain should only be used by bfs, not the recursive dfs
	private ArrayList<UUSearchNode> backchain(UUSearchNode node,
			HashMap<UUSearchNode, UUSearchNode> visited) {
		
		ArrayList<UUSearchNode> returnList = new ArrayList<UUSearchNode>();
		UUSearchNode backtracker = visited.get(node);
		returnList.add(node);
		
		while (backtracker != null) {
			returnList.add(backtracker);
			backtracker = visited.get(backtracker);
		}
		
		return returnList; 
	}
	
	
	protected void resetStats() {
		nodesExplored = 0;
		maxMemory = 0;
	}
	
	
	protected void printStats() {
		System.out.println("Nodes explored during last search:  " + nodesExplored);
		System.out.println("Maximum memory usage during last search " + maxMemory);
	}
	
	
	protected void updateMemory(int currentMemory) {
		maxMemory = Math.max(currentMemory, maxMemory);
	}
	
	
	protected void incrementNodeCount() {
		nodesExplored++;
	}
}