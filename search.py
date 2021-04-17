# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from collections import defaultdict



class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def construct_action(problem, goal, pred):
    action_list = []
    temp = goal
    while temp != problem.getStartState():
        action_list.append(pred[temp][1])
        temp = pred[temp][0]
    action_list.reverse()
    # print(action_list)
    return action_list

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    pred = dict()
    explored = []
    stack = util.Stack()
    stack.push(problem.getStartState())
    while not stack.isEmpty():
        curr_node = stack.pop()
        explored.append(curr_node)
        for child, direction, cost in problem.getSuccessors(curr_node):
            if child not in stack.list and child not in explored:
                pred[child] = (curr_node, direction)
                # print(pred)
                if problem.isGoalState(child):
                    return construct_action(problem, child, pred)
                stack.push(child)
    
    return []

    # util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    pred = dict()
    explored = []
    queue = util.Queue()
    queue.push(problem.getStartState())
    while not queue.isEmpty():
        curr_node = queue.pop()
        explored.append(curr_node)
        for child, direction, cost in problem.getSuccessors(curr_node):
            if child not in queue.list and child not in explored:
                pred[child] = (curr_node, direction)
                if problem.isGoalState(child):
                    return construct_action(problem, child, pred)
                queue.push(child)
    
    return []

    util.raiseNotDefined()

# def breadthFirstSearch(problem):
#     """Search the shallowest nodes in the search tree first."""
#     "*** YOUR CODE HERE ***"

#     from util import Queue
#     fringe = Queue()                        # Fringe to manage which states to expand
#     fringe.push(problem.getStartState())
#     visited = []                            # List to check whether state has already been visited
#     tempPath=[]                             # Temp variable to get intermediate paths
#     path=[]                                 # List to store final sequence of directions 
#     pathToCurrent=Queue()                   # Queue to store direction to children (currState and pathToCurrent go hand in hand)
#     currState = fringe.pop()
#     while not problem.isGoalState(currState):
#         if currState not in visited:
#             visited.append(currState)    
#             successors = problem.getSuccessors(currState)
#             for child,direction,cost in successors:
#                 fringe.push(child)
#                 tempPath = path + [direction]
#                 pathToCurrent.push(tempPath)
#         currState = fringe.pop()
#         path = pathToCurrent.pop()
        
#     return path

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    pred = dict()
    dist = defaultdict(lambda:float("inf"))
    start_state = problem.getStartState()
    dist[start_state] = 0
    explored = []
    queue = util.PriorityQueue()
    queue.push(start_state, 0)
    while not queue.isEmpty():
        curr_node = queue.pop()
        explored.append(curr_node)
        if problem.isGoalState(curr_node):
            return construct_action(problem, curr_node, pred)
        for child, direction, cost in problem.getSuccessors(curr_node):
            if child not in explored:
                if dist[child] > dist[curr_node] + cost:
                    dist[child] = dist[curr_node] + cost
                    pred[child] = (curr_node, direction)
                queue.update(child, dist[curr_node] + cost)
            # print(pred)
    
    return []

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    pred = dict()
    dist = defaultdict(lambda:float("inf"))
    start_state = problem.getStartState()
    dist[start_state] = 0
    explored = []
    queue = util.PriorityQueue()
    queue.push(start_state, 0)
    while not queue.isEmpty():
        curr_node = queue.pop()
        explored.append(curr_node)
        if problem.isGoalState(curr_node):
            return construct_action(problem, curr_node, pred)
        for child, direction, cost in problem.getSuccessors(curr_node):
            if child not in explored:
                if dist[child] > dist[curr_node] + cost:
                    dist[child] = dist[curr_node] + cost
                    pred[child] = (curr_node, direction)
                queue.update(child, dist[curr_node] + cost + heuristic(child, problem))
    
    return []
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
