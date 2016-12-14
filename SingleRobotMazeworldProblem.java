// Written by Brophy Tyree
// This is the first of the three problems - a single robot in a maze
// with barriers. The algorithm uses A* to get the Robot to the goal.


package mazeworld;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SingleRobotMazeworldProblem extends UUSearchProblem {
	
	private int goalx, goaly, mazeWidth, mazeHeight;
	private Maze maze;

	public SingleRobotMazeworldProblem(int width, int height, int gx, int gy, int startx, int starty, int[][] barriers) {
		startNode = new MazeWorldNode(startx, starty, 0);
		goalx = gx;
		goaly = gy;
		mazeWidth = width;
		mazeHeight = height;
		maze = new Maze(mazeWidth, mazeHeight, barriers);
		
		// indicate the start and end goal on the maze
		maze.setPathNode(startx, starty, 's');
		maze.setPathNode(goalx, goaly, 'g');
	}
	
	private class MazeWorldNode implements UUSearchNode {
	
		private int[] state;  // holds (x,y) cord of the current node
		private int depth;  

		public MazeWorldNode(int x, int y, int d) {
			state = new int[2];
			this.state[0] = x;
			this.state[1] = y;	
			depth = d;
		}
		
		// Function returns all possible successors of the current node in an ArrayList
		// This function runs without a heuristic, so the cost of the new node is just 
		// 1 + the cost of the predecessor. It adds the node to successors if the node 
		// is a safe state. This is used for BFS
		public ArrayList<UUSearchNode> getSuccessors() {
			ArrayList<UUSearchNode> successors = new ArrayList<UUSearchNode>();
			int newDepth = depth + 1;
			
			MazeWorldNode topSuccessor = new MazeWorldNode(this.state[0], this.state[1] + 1, newDepth);
			MazeWorldNode bottomSuccessor = new MazeWorldNode(this.state[0], this.state[1] - 1, newDepth);
			MazeWorldNode rightSuccessor = new MazeWorldNode(this.state[0] + 1, this.state[1], newDepth);
			MazeWorldNode leftSuccessor = new MazeWorldNode(this.state[0] - 1, this.state[1], newDepth);
			
			if (topSuccessor.isStateSafe()) { successors.add(topSuccessor); }
			if (bottomSuccessor.isStateSafe()) { successors.add(bottomSuccessor); }
			if (rightSuccessor.isStateSafe()) { successors.add(rightSuccessor); }
			if (leftSuccessor.isStateSafe()) { successors.add(leftSuccessor); }	
			
			return successors;
		}
		
		// getSuccessorsWithHeuristic is used for the A* Search. Like the regular getSuccessors,
		// it produces all legal and safe successors for the current node (up, down, left, right).
		// the cost of the new state is (predecessors cost + 1 + the heuristic of the new state)
		public ArrayList<UUSearchNode> getSuccessorsWithHeuristic() {
			ArrayList<UUSearchNode> successors = new ArrayList<UUSearchNode>();
			int newDepth = depth + 1;
			
			MazeWorldNode topSuccessor = new MazeWorldNode(this.state[0], this.state[1] + 1, 
					newDepth + calculateManhattanHeuristic(this.state[0], this.state[1] + 1));
			
			MazeWorldNode bottomSuccessor = new MazeWorldNode(this.state[0], this.state[1] - 1, 
					newDepth + calculateManhattanHeuristic(this.state[0], this.state[1] - 1));
			
			MazeWorldNode rightSuccessor = new MazeWorldNode(this.state[0] + 1, this.state[1], 
					newDepth + calculateManhattanHeuristic(this.state[0] + 1, this.state[1]));
			
			MazeWorldNode leftSuccessor = new MazeWorldNode(this.state[0] - 1, this.state[1], 
					newDepth + calculateManhattanHeuristic(this.state[0] - 1, this.state[1]));
			
			if (topSuccessor.isStateSafe()) { successors.add(topSuccessor); }
			if (bottomSuccessor.isStateSafe()) { successors.add(bottomSuccessor); }
			if (rightSuccessor.isStateSafe()) { successors.add(rightSuccessor); }
			if (leftSuccessor.isStateSafe()) { successors.add(leftSuccessor); }	
			
			return successors;
		}
		
		// for the single robot, the manhattan heuristic is a simple heuristic
		// that produces optimal paths. It is simply the right triangle x and y 
		// distance from the node to the goal;
		public int calculateManhattanHeuristic(int x, int y) {
			int xDist = Math.abs(goalx - x);
			int yDist = Math.abs(goaly - y);
			
			return (xDist + yDist);
		}
		
		// test to see if the total goal has been accomplished
		@Override
		public boolean goalTest() {
			if (this.state[0] == goalx && this.state[1] == goaly) {
				return true;
			} else {
				return false; 
			}
		}
		
		// check to see if the individual robot has reached its goal (not necessary 
		// for the single robot.
		public boolean individualGoalTest() {
			if (this.state[0] == goalx && this.state[1] == goaly) {
				return true;
			} else {
				return false; 
			}
		}
		
		// return the depth of the current node
		public int getDepth() {
			return this.depth;
		}
		
		// return the x cord of the current node
		public int getx() {
			return this.state[0];
		}
		
		// return the y cord of the current node
		public int gety() {
			return this.state[1];
		}
		
		// checks if the new state (successor) is safe - if the x and y coordinates are greater
		// than zero and less than the width/height of the maze and if the new state does not have a 
		// barrier, then it is safe
		private boolean isStateSafe() {
			if (this.state[1] < mazeHeight && this.state[1] >= 0 &&this.state[0] >= 0 && 
					this.state[0] < mazeWidth && maze.getMazeState(this.state[0], this.state[1]) != '#') {
				return true;
			} else {
				return false; 
			}
		}

		
		@Override
		public boolean equals(Object other) {
			return Arrays.equals(state, ((MazeWorldNode) other).state);
		}

		@Override
		public int hashCode() {
			return state[0] * 100 + state[1] * 10;
		}
	}
	
	// this method shows the path after it has been found by a serach algorithm.
	// given the path as a parameter, it produces a visualization on the maze
	public void showPath(List<UUSearchNode> path) {
		for (int i = 0; i < path.size(); i++) {
			UUSearchNode current = path.get(i);
			int x = current.getx();
			int y = current.gety();
			maze.setPathNode(x, y, 'o');
		}
		maze.displayMaze();
		System.out.println("");
		System.out.println("The path has a length of " + (path.size()));
	}
	
	
	// display the current map
	public void display() {
		maze.displayMaze();
	}

	public static void main(String[] args) {
		
		int[][] barriers1 = { {2,6}, {3,6}, {3,5}, {4,5}, {4,3}, {3,2}, {3,3}, {2,1}, {6,0}, {5,0} };
		SingleRobotMazeworldProblem problem1 = new SingleRobotMazeworldProblem(7, 7, 5, 5, 0, 0, barriers1);
		problem1.display();
		System.out.println("Finding path using BFS");
		List<UUSearchNode> path = problem1.breadthFirstSearch();
		System.out.println("");
		System.out.println("");
		problem1.showPath(path);
		System.out.println("");
		System.out.println("");
		
		int[][] barriers3 = { {2,6}, {3,6}, {3,5}, {4,5}, {4,3}, {3,2}, {3,3}, {2,1}, {6,0}, {5,0} };
		SingleRobotMazeworldProblem problem3 = new SingleRobotMazeworldProblem(7, 7, 5, 5, 0, 0, barriers3);
		problem3.display();
		List<UUSearchNode> path3 = problem3.AStarSearch();
		System.out.println("");
		System.out.println("");
		problem3.showPath(path3);
		System.out.println("");
		System.out.println("");
		
		int[][] barriers2 = { {3,1}, {3,2}, {3,3}, {5,1}, {5,2}, {5,3}, {6,3}, {7,3}, {8,3}, {2,3}, {1,3}, {0,3}, {0,4}, {0,5}, {1,5}, {2,5}, {3,5}, {4,5}, {5,5}, {6,5}, {7,5}, {8,5}, {8,4}, {8,3}};
		SingleRobotMazeworldProblem problem2 = new SingleRobotMazeworldProblem(10, 10, 9, 9, 5, 4, barriers2);
		problem2.display();
		List<UUSearchNode> path2 = problem2.AStarSearch();
		System.out.println("");
		System.out.println("");
		problem2.showPath(path2);
	}	
}