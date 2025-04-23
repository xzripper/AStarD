// A* (A-Star) search algorithm for 2D grids by Evan (xzripper): https://github.com/xzripper/AStarD

module AStarD.AStar;

import std.math : abs, pow, sqrt;

import std.algorithm : reverse;

import AStarD.DHeap.Heap;

alias ASPath = int[2][], Grid2d = int[][], Position2D = int[2];

enum Heuristic { MANHATTAN, EUCLIDEAN, OCTILE, CHEBYSHEV }

private static immutable STRAIGHT_DIRECTIONS = [[-1, 0], [1, 0], [0, -1], [0, 1]];
private static immutable STRAIGHT_DIAGONAL_DIRECTIONS = [[-1, 0], [1, 0], [0, -1], [0, 1], 
                                                        [-1, -1], [-1, 1], [1, -1], [1, 1]];

private alias Node = float[3];

private float _CalculateHeuristic(
    Position2D p_Position0,
    Position2D p_Position1,
    Heuristic p_Heuristic ) @safe
{
    if( p_Heuristic == Heuristic.MANHATTAN ) {
        return abs( p_Position0[0] - p_Position1[0] ) + abs( p_Position0[1] - p_Position1[1] );
    } else if( p_Heuristic == Heuristic.EUCLIDEAN ) {
        return sqrt( cast(float) (pow( p_Position0[0] - p_Position1[0], 2 ) + 
                                  pow( p_Position0[1] - p_Position1[1], 2 )) );
    } else if( p_Heuristic == Heuristic.OCTILE ) {
        float t_DX = abs( p_Position0[0] - p_Position1[0] );
        float t_DY = abs( p_Position0[1] - p_Position1[1] );

        return (t_DX > t_DY ? t_DX : t_DY) + (sqrt( 2.0f ) - 1) * (t_DY > t_DX ? t_DY : t_DX);
    } else if( p_Heuristic == Heuristic.CHEBYSHEV ) {
        float t_MaxOp0 = abs( p_Position0[0] - p_Position1[0] );
        float t_MaxOp1 = abs( p_Position0[1] - p_Position1[1] );

        return (t_MaxOp0 > t_MaxOp1 ? t_MaxOp0 : t_MaxOp1);
    } else { assert( 0 ); }
}

ASPath AStar(
    Grid2d p_Grid,
    Position2D p_Start,
    Position2D p_Target,
    Heuristic p_Heuristic,
    bool p_AllowDiagonalMovement ) @safe
{
    if ( p_Grid.length <= 1 ) { return []; }

    Heap!Node t_NodesHeap = Heap!Node( HeapType.MIN_HEAP );

    t_NodesHeap.HeapPush( [0, p_Start[0], p_Start[1]] );

    int[2][int[2]] t_CameFrom;

    float[int[2]] t_GScores = [p_Start: 0];
    float[int[2]] t_FScores = [p_Start: _CalculateHeuristic( p_Start, p_Target, p_Heuristic )];

    while( !t_NodesHeap.Empty() )
    {
        float[3] t_HeapPopOut = t_NodesHeap.HeapPop();

        int[2] t_CurrentNodePos = [cast(int) t_HeapPopOut[1], cast(int) t_HeapPopOut[2]];

        if( t_CurrentNodePos == p_Target )
        {
            int[2][] t_OutPath;

            while( t_CurrentNodePos in t_CameFrom )
            {
                t_OutPath ~= t_CurrentNodePos;

                t_CurrentNodePos = t_CameFrom[t_CurrentNodePos];
            }

            t_OutPath ~= p_Start;

            return reverse( t_OutPath );
        }

        foreach( int[2] t_Direction; (p_AllowDiagonalMovement ? STRAIGHT_DIAGONAL_DIRECTIONS : STRAIGHT_DIRECTIONS) )
        {
            int[2] t_Neighbour = [t_CurrentNodePos[0] + t_Direction[0], t_CurrentNodePos[1] + t_Direction[1]];

            if( 0 <= t_Neighbour[0] && t_Neighbour[0] < p_Grid.length &&
                0 <= t_Neighbour[1] && t_Neighbour[1] < p_Grid[0].length )
            {
                if( p_Grid[t_Neighbour[0]][t_Neighbour[1]] == 1 ) { continue; }

                float t_TentativeGScore = t_GScores[t_CurrentNodePos] + (p_AllowDiagonalMovement ? 
                        (t_Direction[0] != 0 && t_Direction[1] != 0 ? sqrt( 2.0f ) : 1.0f) : 1);

                if( t_Neighbour !in t_GScores || t_TentativeGScore < t_GScores[t_Neighbour] )
                {
                    t_CameFrom[t_Neighbour] = t_CurrentNodePos;

                    t_GScores[t_Neighbour] = t_TentativeGScore;
                    t_FScores[t_Neighbour] = t_TentativeGScore + _CalculateHeuristic( t_Neighbour,
                                                                                      p_Target,
                                                                                      p_Heuristic );

                    t_NodesHeap.HeapPush( [t_FScores[t_Neighbour], t_Neighbour[0], t_Neighbour[1]] );
                }
            }
        }
    }

    return [];
}
