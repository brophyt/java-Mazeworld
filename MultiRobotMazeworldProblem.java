// Written by Brophy Tyree
// This is the second variation of the problem - Multi robot maze
// The algorithm has to make the robots cooperate to get to their
// respective goals in an optimal path 

package mazeworld;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class MultiRobotMazeworldProblem extends UUSearchProblem {
	
	private int goalx, goaly, mazeWidth, mazeHeight;
	
	// array contains the letters associated with each robot index
	private char[] robotsLetters;      
	
	// arrays contain the location of the barriers and the goals
	private int[][] barriers, robotsGoals;
	private Maze maze;

	public MultiRobotMazeworldProblem(int width, int height, int[][] rg, int[][] robotsStart, int[][] b) {
		mazeWidth = width;
		mazeHeight = height;
		barriers = b;
		robotsGoals = rg;
		
		maze = new Maze(mazeWidth, mazeHeight, barriers);
		startNode = new MultiMazeWorldNode(0, 0, robotsStart);
		robotsLetters = new char[robotsStart.length];
		
		// produce the Robots letters array, where index 0 -> 'A', index 1 -> 'B".....
		char currentLetter = 'A';
		for (int i = 0; i < robotsStart.length; i++) {
			maze.setPathNode(robotsStart[i][0], robotsStart[i][1], currentLetter);
			robotsLetters[i] = currentLetter;
			currentLetter++;
		}		
		
		// display on the maze where each goal is.
		char currentLetter2 = 'a';
		for (int i = 0; i < robotsStart.length; i++) {
			maze.setPathNode(robotsGoals[i][0], robotsGoals[i][1], currentLetter2);
			currentLetter2++;
		}	

	}
	
	
	private class MultiMazeWorldNode implements UUSearchNode {
		
		// maintains the locations of all of the robots
		private int[][] state; 
		// maintains the current cost as well as the index of the robot that is moving
		private int cost, robotToMove; 

		public MultiMazeWorldNode(int d, int robotIndex, int[][] robotPositions) {
			// wrap around if the last robot just moved
			if (robotIndex == robotPositions.length) {
				robotToMove = 0;
			} else {
				robotToMove = robotIndex;
			}
			
			state = robotPositions;
			cost = d;
		}		
		
		// There are five possible moves per state - up, down, left, right and skip.
		// this method creates a new state for every move (the state is the locations of all 
		// robots. Then it calculates the heuristic of the new state. Finally it checks to make 
		// sure the states are safe. If there are no safe or legal moves available, the method skips 
		// to the next robot, who picks a move at the same cost. 
		public ArrayList<UUSearchNode> getSuccessorsWithHeuristic() {
			// if the current robot is at its goal, skip the turn
			while (this.individualGoalTest()) {
				robotToMove++;
				if (robotToMove == state.length) {
					robotToMove = 0;
				}
			}
			
			int newDepth = cost + 1;
			while (true) {
				ArrayList<UUSearchNode> successors = new ArrayList<UUSearchNode>();
				
				int[][] newStateUpMove = createNewState(this.state[robotToMove][0], this.state[robotToMove][1] + 1);
				int depth1 = newDepth + calculateHeuristic(this.state[robotToMove][0], this.state[robotToMove][1] + 1);
				MultiMazeWorldNode topSuccessor = new MultiMazeWorldNode(depth1, robotToMove + 1, newStateUpMove);
				
				int[][] newStateDownMove = createNewState(this.state[robotToMove][0], this.state[robotToMove][1] - 1);
				int depth2 = newDepth + calculateHeuristic(this.state[robotToMove][0], this.state[robotToMove][1] - 1);
				MultiMazeWorldNode bottomSuccessor = new MultiMazeWorldNode(depth2, robotToMove + 1, newStateDownMove);
				
				int[][] newStateRightMove = createNewState(this.state[robotToMove][0] + 1, this.state[robotToMove][1]);
				int depth3 = newDepth + calculateHeuristic(this.state[robotToMove][0] + 1, this.state[robotToMove][1]);
				MultiMazeWorldNode rightSuccessor = new MultiMazeWorldNode(depth3, robotToMove + 1, newStateRightMove);
				
				int[][] newStateLeftMove = createNewState(this.state[robotToMove][0] - 1, this.state[robotToMove][1]);
				int depth4 = newDepth + calculateHeuristic(this.state[robotToMove][0] - 1, this.state[robotToMove][1]);
				MultiMazeWorldNode leftSuccessor = new MultiMazeWorldNode(depth4, robotToMove + 1, newStateLeftMove);
				
				if (topSuccessor.isStateSafe()) { successors.add(topSuccessor); }
				if (bottomSuccessor.isStateSafe()) { successors.add(bottomSuccessor); }
				if (rightSuccessor.isStateSafe()) { successors.add(rightSuccessor); }
				if (leftSuccessor.isStateSafe()) { successors.add(leftSuccessor); }	
				
				
				if (successors.size() != 0) {
					return successors;
				} else {
					robotToMove++;
					if (robotToMove == state.length) {
						robotToMove = 0;
					}
				}
			}
		}
		
		// not applicable for multiple robots (always uses heuristic)
		public ArrayList<UUSearchNode> getSuccessors() {
			return null;
		}
		
		// method creates a new state for the successor. It takes the x and y 
		// coordinate of the robot that just moved and creates a new 2d array 
		// state with the robots new location
		public int[][] createNewState(int x, int y) {
			int[][] returnState = new int[state.length][2];
			int[] newLocation = {x,y};
			
			for (int i = 0; i < state.length; i++) {
				for (int j = 0; j < state[i].length; j++) {
					returnState[i][j] = state[i][j];
				}
			}
			returnState[robotToMove] = newLocation;
			return returnState;
		}
		
		// This method checks to see if the new state is safe. if the robot that just moved
		// has an x coordinate that is greater than 0 and less than mazewidth and the y coord
		// is greater than 0 and less than mazeheight, and the new state does not have a robot
		// colliding with a barrier, and the new state does not have two robots colliding, the
		// state is safe. 
		private boolean isStateSafe() {
			int lastRobot = robotToMove - 1;
			if (lastRobot < 0) {
				lastRobot = this.state.length - 1;
			}
			if (this.state[lastRobot][1] < mazeHeight && this.state[lastRobot][1] >= 0 && 
					this.state[lastRobot][0] >= 0 && this.state[lastRobot][0] < mazeWidth && 
					maze.getMazeState(this.state[lastRobot][0], this.state[lastRobot][1]) != '#') {
				
				for (int i = 0; i < state.length; i++) {
					for (int j = i+1; j < state.length; j++) {
						if (Arrays.equals(state[i], state[j]))  { return false; }
					}
				}
				
				return true;
			} else {
				return false; 
			}
		}
		 
		
		// the heuristic for this search is the sum of the distances of every
		// robot to its goal
		public int calculateHeuristic(int x, int y) {
			int heuristic = 0;
			int newX = Math.abs(robotsGoals[robotToMove][0] - x);
			int newY = Math.abs(robotsGoals[robotToMove][1] - y);
			heuristic += newX + newY;
			for (int i = 0; i < state.length; i++) {
				if (i != robotToMove) {
					int xDist = Math.abs(robotsGoals[i][0] - state[i][0]);
					int yDist = Math.abs(robotsGoals[i][1] - state[i][1]); 
					heuristic += xDist + yDist;
				}
			}		
			return (heuristic);
		}
		
		// the total goal test - if every robot is at its goal state, the goal has been reached
		@Override
		public boolean goalTest() {
			for (int i = 0; i < state.length; i++) {
				if (!(this.state[i][0] == robotsGoals[i][0] && this.state[i][1] == robotsGoals[i][1])) {
					return false; 
				}
			}
			return true;
		}
		
		// if the one robot is at its goal state
		public boolean individualGoalTest() {
			if (this.state[robotToMove][0] == robotsGoals[robotToMove][0] && this.state[robotToMove][1] == robotsGoals[robotToMove][1]) {
				return true;
			} else {
				return false; 
			}
		}
		
		public int getDepth() {
			return this.cost;
		}
		
		public int getx() {
			return this.state[robotToMove][0];
		}
		
		public int gety() {
			return this.state[robotToMove][1];
		}
		
		// do the 2d arrays have equal values?
		@Override
		public boolean equals(Object other) {
			for (int i = 0; i < state.length; i++){
				if (!Arrays.equals(state[i], ((MultiMazeWorldNode) other).state[i])) {
					return false; 
				}
			}
			return true;
		}

		// this hashcode makes sure that each individual state is hashed into the hashtable
		// with no collisions. We have a multiplier that increases by a factor of 100 for every
		// index in the state array
		@Override
		public int hashCode() {
			int returnInt = 0;
			int multiplier = 10;
			for (int i = 0; i < this.state.length; i++) {
				returnInt += this.state[i][0] * 10 * multiplier + this.state[i][1] * 1 * multiplier;
				multiplier *= 100;
			}
			return returnInt;
		}
	}
	
	
	// this function displays the path
	public void showPath(List<UUSearchNode> path) {
		for (int i = path.size() - 1; i >= 0; i--) {
			MultiMazeWorldNode current = (MultiMazeWorldNode) path.get(i);
			maze = new Maze(mazeWidth, mazeHeight, barriers);
			int[][] currentState = current.state;
			
			char currentLetter = 'A';
			for (int j = 0; j < currentState.length; j++) {
				maze.setPathNode(currentState[j][0], currentState[j][1], currentLetter);
				currentLetter++;
			}	
			
			maze.displayMaze();
			System.out.println("");
			System.out.println("");
		}
	}
	
	public void display() {
		maze.displayMaze();
	}

	public static void main(String[] args) {
		int[][] barriers1 = { {4,0}, {4,2}, {5,0}, {6,0}, {7,0}, {8,0}, {9,0}, {10,0}, {11,0}, {12,0}, 
				{5,2}, {6,2}, {7,2}, {8,2}, {9,2}, {10,2}, {11, 2}, {12,2}, {14,1} };
		int[][] robots1 = { {10,1}, {9,1}, {11,1} };
		int[][] goals1 = { {16,2}, {15,2}, {14,2} };

		MultiRobotMazeworldProblem problem1 = new MultiRobotMazeworldProblem(18, 3, goals1, robots1, barriers1);
		problem1.display();
		List<UUSearchNode> path = problem1.AStarSearch();
		System.out.println("");
		problem1.showPath(path);
		
		System.out.println("");
		System.out.println("-------------------------------");
		System.out.println("");

		int[][] barriers2 = { {3,1}, {3,2}, {3,3}, {5,1}, {5,2}, {5,3}, {6,3}, {7,3}, {8,3}, {2,3}, {1,3}, 
				{0,3}, {0,4}, {0,5}, {1,5}, {2,5}, {3,5}, {4,5}, {5,5}, {6,5}, {7,5}, {8,5}, {8,4}, {8,3}};
		int[][] robots2 = { {5,4}, {4,4}, {3,4} };
		int[][] goals2 = { {9,8}, {8,8}, {7,8} };
			
		MultiRobotMazeworldProblem problem2 = new MultiRobotMazeworldProblem(10, 10, goals2, robots2, barriers2);
		problem2.display();
		List<UUSearchNode> path2 = problem2.AStarSearch();
		System.out.println("");
		problem2.showPath(path2);
		
		System.out.println("");
		System.out.println("-------------------------------");
		System.out.println("");
		
		int[][] barriers3 = { {4,0}, {4,2}, {5,0}, {6,0}, {7,0}, {8,0}, {9,0}, {10,0}, {11,0}, {12,0}, 
				{5,2}, {6,2}, {7,2}, {8,2}, {9,2}, {10,2}, {11, 2}, {12,2} };
		int[][] robots3 = { {10,1}, {9,1} };
		int[][] goals3 = { {1, 1}, {14,1} };

		MultiRobotMazeworldProblem problem3 = new MultiRobotMazeworldProblem(18, 3, goals3, robots3, barriers3);
		problem3.display();
		List<UUSearchNode> path3 = problem3.AStarSearch();
		System.out.println("");
		problem3.showPath(path3);
		
		int[][] barriers4 = {  };
		int[][] robots4 = { {0,1}, {0,9}, {0,5} };
		int[][] goals4 = { {9,9}, {9,3}, {9,1} };

		MultiRobotMazeworldProblem problem4 = new MultiRobotMazeworldProblem(10, 10, goals4, robots4, barriers4);
		problem4.display();
		List<UUSearchNode> path4 = problem4.AStarSearch();
		System.out.println("");
		problem4.showPath(path4);
		
		System.out.println("");
		System.out.println("-------------------------------");
		System.out.println("");
	}
	
}
	