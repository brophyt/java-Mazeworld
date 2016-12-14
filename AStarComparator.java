// Written by Brophy Tyree
// This is a comparator for the A* Priority Queue.
// it compares the two Search Nodes by their depth, returning the lowest 
// depth in the queue

package mazeworld;

import java.util.Comparator;

import mazeworld.UUSearchProblem.UUSearchNode;

public class AStarComparator implements Comparator<UUSearchNode>
{
    @Override
    public int compare(UUSearchNode x, UUSearchNode y)
    {
        if (x.getDepth() < y.getDepth())
        {
            return -1;
        }
        if (x.getDepth() > y.getDepth())
        {
            return 1;
        }
        return 0;
    }
}