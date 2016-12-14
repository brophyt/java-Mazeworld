// written by Brophy Tyree
// this is the class that holds all the information about the state of the maze.
// this maze class is used for the single robot problem, the multi robot problem
// and the blind robot problem

package mazeworld;

import java.util.ArrayList;
import java.util.Arrays;

public class Maze {
	private int width;
	private int height; 
	private int[][] barriers;    // holds the (x,y) coordinates of every barrier
	private char[][] mazeState;  // holds the characters at every index of the maze
	
	private final static char EMPTY = '.';
	private final static char BARRIER = '#';
	
	public Maze(int w, int h, int[][] b) {
		width = w;
		height = h;
		barriers = b;
		mazeState = new char[w][h];
		
		// create an empty maze
		for (int i = 0; i < width; i++) {
			for (int n = 0; n < height; n++) {
				mazeState[i][n] = EMPTY;
			}
		}
		
		// fill in the appropriate barriers
		for (int i = 0; i < barriers.length; i++) {
			mazeState[barriers[i][0]][barriers[i][1]] = BARRIER;
		}
	}
	
	// returns the current state of the maze
	public char getMazeState(int x, int y) {
		return mazeState[x][y];
	}
	
	// set the given (x,y) coordinate of the maze to the given character
	public void setPathNode(int x, int y, char c) {
		mazeState[x][y] = c;
	}
	
	// display the maze given the current state
	public void displayMaze() {
		for (int h = (height - 1); h >= 0; h--) {
			for (int w = 0; w < width; w++) {
				System.out.print(mazeState[w][h] + " ");
			}
			System.out.print("\n");
		}
	}
	
	public static void main(String[] args) {
		
		int[][] barriers1 = { {1,2}, {3,4}, {1,3}, {5,5} };
		Maze maze1 = new Maze(6, 6, barriers1);
		maze1.displayMaze();	
	}
}