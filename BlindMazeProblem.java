// Written by Brophy Tyree
// This is the third portion of the lab - a blind robot in a maze.
// The robot does not know where it begins and does not know when it 
// hits a wall or obstacle 

package mazeworld;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class BlindMazeProblem extends UUSearchProblem {
	
	private int goalx, goaly, mazeWidth, mazeHeight, startx, starty;
	
	// two arrayLists of the possible x coordinates and the possible y coordinates
	// array list used so that states can be removed and added easily
	private ArrayList<Integer> possibleX, possibleY;
	private Maze maze;

	public BlindMazeProblem(int width, int height, int startX, int startY, int goalX, int goalY, int[][] barriers) {
		mazeWidth = width;
		mazeHeight = height;
		goalx = goalX;
		goaly = goalY;
		startx = startX;
		starty = startY;
		
		maze = new Maze(mazeWidth, mazeHeight, barriers);
		
		possibleX = new ArrayList<Integer>();
		possibleY = new ArrayList<Integer>();
		 
		// the original state is all x coordinates and y coordinates
		for (int w = 0; w < mazeWidth; w++) {
			possibleX.add(w);
		}
		
		for (int h = 0; h < mazeHeight; h++) {
			possibleY.add(h);
		}
		
		startNode = new BlindSearchNode(possibleX, possibleY, startX, startY, "");
		maze.setPathNode(startX, startY, 's');
		maze.setPathNode(goalx, goaly, 'g');
	}
	
	
	private class BlindSearchNode implements UUSearchNode {
	
		private ArrayList<Integer> stateX;  
		private ArrayList<Integer> stateY;
		int cost, topMostOption, bottomMostOption, rightMostOption, leftMostOption, x, y;
		String direction;

		public BlindSearchNode(ArrayList<Integer> possibleX, ArrayList<Integer> possibleY, int startX, int startY, String theDirection) {
			stateX = possibleX;
			stateY = possibleY;
			cost = this.calculateHeuristic();
			x = startX;
			y = startY;
			direction = theDirection;
			
			if (x < 0) {
				x = 0;
			} else if (x >= mazeWidth) {
				x = mazeWidth - 1;
			}
			
			if (y < 0) {
				y = 0;
			} else if (y >= mazeWidth) {
				y = mazeWidth - 1;
			}
			
			// variables hold the topMost, bottomMost, leftMost, and rightMost
			// coordinates in the current belief state. These values are used when 
			// a robot moves (removed or added)
			topMostOption = 0;
			bottomMostOption = mazeHeight;
			rightMostOption = 0;
			leftMostOption = mazeWidth;
			for (int i = 0; i < stateX.size(); i++) {
				int x = stateX.get(i);
				rightMostOption = Math.max(x, rightMostOption);
				leftMostOption = Math.min(x, leftMostOption);
			}
			for (int i = 0; i < stateY.size(); i++) {
				int y = stateY.get(i);		
				topMostOption = Math.max(y, topMostOption);
				bottomMostOption = Math.min(y, bottomMostOption);
			}
		}		
		
		// method gets all successors of the current state by either moving the robot north, south, 
		// east, or west
		public ArrayList<UUSearchNode> getSuccessorsWithHeuristic() {
			
			ArrayList<UUSearchNode> successors = new ArrayList<UUSearchNode>();
			
			ArrayList<Integer> newStateUpMove = this.createNewState("up");
			BlindSearchNode topSuccessor = new BlindSearchNode(stateX, newStateUpMove, x, y+1, "north");
			
			ArrayList<Integer> newStateDownMove = this.createNewState("down");
			BlindSearchNode bottomSuccessor = new BlindSearchNode(stateX, newStateDownMove, x, y-1, "south");
			
			ArrayList<Integer> newStateRightMove = this.createNewState("right");
			BlindSearchNode rightSuccessor = new BlindSearchNode(newStateRightMove, stateY, x+1, y, "east");
			
			ArrayList<Integer> newStateLeftMove = this.createNewState("left");
			BlindSearchNode leftSuccessor = new BlindSearchNode(newStateLeftMove, stateY, x-1, y, "west");
			
			if (topSuccessor.isStateSafe()) { successors.add(topSuccessor); }
			if (bottomSuccessor.isStateSafe()) { successors.add(bottomSuccessor); }
			if (rightSuccessor.isStateSafe()) { successors.add(rightSuccessor); }
			if (leftSuccessor.isStateSafe()) { successors.add(leftSuccessor); }	
			
			return successors;
		}
		
		// not used for blind robot
		public ArrayList<UUSearchNode> getSuccessors() {
			return null;
		}
		
		// this method creates a new ArrayList of belief states, depending
		// on the direction of the robot move. when moving north, the bottom-
		// most y coordinate is removed from the belief state and 
		// (1 + the top-most) is added to the belief state if the top-most 
		// is not already the highest possible y coordinate
		public ArrayList<Integer> createNewState(String direction) {
			ArrayList<Integer> returnState = new ArrayList<Integer>();
			
			if (direction.equals("up")) {
				for (int i = 0; i < stateY.size(); i++) {
					if (stateY.get(i) != bottomMostOption) {
						returnState.add(stateY.get(i));
					}
				}
				if (topMostOption < mazeHeight && !returnState.contains(topMostOption)) {
					returnState.add(topMostOption);
				}
			} 
			else if (direction.equals("down")) {
				for (int i = 0; i < stateY.size(); i++) {
					if (stateY.get(i) != topMostOption) {
						returnState.add(stateY.get(i));
					}
				}
				if (bottomMostOption >= 0 && !returnState.contains(bottomMostOption)) {
					returnState.add(bottomMostOption);
				}
			} 
			else if (direction.equals("left")) {
				for (int i = 0; i < stateX.size(); i++) {
					if (stateX.get(i) != rightMostOption) {
						returnState.add(stateX.get(i));
					}
				}	
				if (leftMostOption >= 0 && !returnState.contains(leftMostOption)) {
					returnState.add(leftMostOption);
				}
			} 
			else if (direction.equals("right")) {
				for (int i = 0; i < stateX.size(); i++) {
					if (stateX.get(i) != leftMostOption) {
						returnState.add(stateX.get(i));
					}
				}
				if (rightMostOption < mazeWidth && !returnState.contains(rightMostOption)) {
					returnState.add(rightMostOption);
				}
			}			
			return returnState;
		}
		
		
		private boolean isStateSafe() {
			if (this.stateX.size() != 0 && this.stateY.size() != 0 ) {
				if (maze.getMazeState(this.getx(), this.gety()) != '#') {
					return true;
				}
			} 
			return false;
		}
		
		// the heuristic for this problem is the number of states in the belief 
		// state + the multiplier(10) to the power of the distance of that belief 
		// state to the goal.
		public int calculateHeuristic() {
			int heuristic = 0;
			int multiplier = 10; 
			for (int i = 0; i < stateX.size(); i++) {
				int distanceToGoalX = Math.abs(goalx - stateX.get(i));
				heuristic++;
				heuristic += Math.pow(multiplier, distanceToGoalX);
			}
			for (int i = 0; i < stateY.size(); i++) {
				int distanceToGoalY = Math.abs(goalx - stateY.get(i));
				heuristic++;
				heuristic += Math.pow(multiplier, distanceToGoalY);
			}
			return heuristic;
		}
		
		// the goal is reached when there is only one state in the belief state and it 
		// is the goal coordinates
		@Override
		public boolean goalTest() {
			if (this.stateX.size() == 1 && this.stateY.size() == 1) {
				if (this.stateX.get(0) == goalx && this.stateY.get(0) == goaly) {
					return true;
				}
			}
			return false;
		}
		
		
		public boolean individualGoalTest() {
			return false;
		}
		
		
		public int getDepth() {
			return this.cost;
		}
		
		public int getx() {
			return this.x;
		}
		
		public int gety() {
			return this.y;
		}
		
		public String getDirection() {
			return this.direction;
		}
		
		
		@Override
		public boolean equals(Object other) {
			if (((BlindSearchNode) other).stateX.size() == this.stateX.size() && 
					((BlindSearchNode) other).stateY.size() == this.stateY.size()) {
				for (int i = 0; i < stateX.size(); i++){
					if (stateX.get(i) != ((BlindSearchNode) other).stateX.get(i)) {
						return false; 
					}
				}			
				for (int j = 0; j < stateY.size(); j++){
					if (stateY.get(j) != ((BlindSearchNode) other).stateY.get(j)) {
						return false; 
					}
				}		
				return true;
			}
			return false;
		}

		
		@Override
		public int hashCode() {
			int returnInt = 0;
			int multiplier = 10;
			for (int i = 0; i < this.stateX.size(); i++) {
				returnInt += this.stateX.get(i) * multiplier;
				multiplier *= 10;
			}
			for (int j = 0; j < this.stateY.size(); j++) {
				returnInt += this.stateY.get(j) * multiplier;
				multiplier *= 10;
			}
			return returnInt;
		}
	}
	
	
	public void showPath(List<UUSearchNode> path) {
		String pathString = "The whole path is: ";
		for (int i = path.size() - 2; i >= 0; i--) {
			BlindSearchNode current = (BlindSearchNode) path.get(i);
			int x = current.getx();
			int y = current.gety();
			String direction = current.getDirection();
			pathString += direction + ", ";
			char symbol = '^';
			if (direction.equals("north")) {
				symbol = '^';
			} else if (direction.equals("south")) {
				symbol = 'v';
			} else if (direction.equals("east")) {
				symbol = '>';
			} else if (direction.equals("west")) {
				symbol = '<';
			}
			
			maze.setPathNode(x, y, symbol);
			maze.displayMaze();
			
			System.out.println("The robot just moved " + direction);
			System.out.print("\nThe current belief state is: \n");
			System.out.print("Potential x coordinates: (");
			for (int j = 0; j < current.stateX.size(); j++) {
				if (j != 0) {
					System.out.print(", ");
				}
				System.out.print(current.stateX.get(j));
			}
			System.out.print(")\n");
			
			System.out.print("Potential y coordinates: (");
			for (int j = 0; j < current.stateY.size(); j++) {
				if (j != 0) {
					System.out.print(", ");
				}
				System.out.print(current.stateY.get(j));
			}
			System.out.print(")\n");
		}
		System.out.print(pathString);
	}
	
	public void display() {
		maze.displayMaze();
	}

	public static void main(String[] args) {
		int[][] barriers1 = { {1,2} };

		BlindMazeProblem problem1 = new BlindMazeProblem(6, 6, 1, 1, 4, 4, barriers1);
		problem1.display();
		List<UUSearchNode> path = problem1.AStarSearch();
		System.out.println("");
		problem1.showPath(path);
		
		System.out.println("");
		System.out.println("-------------------------------");
		System.out.println("");
	}
}
	