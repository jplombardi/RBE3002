import math
import rospy, tf
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PointStamped, PoseStamped, Twist, Pose
from move_base_msgs.msg import MoveBaseActionResult


# Team Crug, RBE3002 Final Project Code
# Created by Joseph Lombardi, Brandon Kelly, William Godsey
# In short, script creates node, reads map data on callback.
# Creates our own version of map, uses it to find frontiers, and publishes centroid of largest.



class Node():  # class for storing node information
    def __init__(self, x, y, cost, row, col):
        self.x = x
        self.y = y
        self.cost = cost
        self.frontierNumber = -1  # frontier number represents its status or group
        # a -1 means unkown, 0 means is a frontier, 1+ is group
        self.row = row
        self.col = col


class FinalProject():
    def __init__(self):
        # Define Map subscribers/publishers
        self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.readOccupancyGridMap, queue_size=1)
        self.pubFrontier = rospy.Publisher("/frontier", GridCells, queue_size=10)
        self.pubTheFrontier = rospy.Publisher("/theFrontier", GridCells, queue_size=10)

        ## Movement specific publishers/subscribers
        self.spinPub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,
                                       queue_size=10)  # Publisher for commanding robot motion
        self.goalWaiter = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.explorationStatus,
                                           queue_size=10)
        self.navGoalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # set up control parameters
        self.hz = 10
        self.spinRate = rospy.Rate(self.hz)
        self.frameID = "map"

        # set up variables that may be referenced externally
        self.exploring = True
        self.frontierThreshold = 10
        self.frontierToMove = []
        self.goalx = 0
        self.goaly = 0
        self.waitForMap = True

        # set up of function variables
        self.freeThreshold = 0
        self.occupiedThreshold = 10

        rospy.sleep(rospy.Duration(1, 0))  # wait for a moment

    # Function brings in map and gives us our nodes with cost and (x,y) values
    # Acts upon callback of the subscriber of subMap
    def readOccupancyGridMap(self, map):
        mapRows = map.info.height
        mapCols = map.info.width
        self.resolution = map.info.resolution
        self.origin = map.info.origin
        grid = [[map.data[(row * mapCols) + col] for col in range(mapCols)] for row in range(mapRows)]

        self.createMap(grid)  # pass data from occupancy grid to create a map of nodes

    # Creates map of nodes given an occupancy matrix, returns theMap
    def createMap(self, occupancyMatrix):
        self.theMap = [[None for col in range(len(occupancyMatrix[row]))] for row in range(len(occupancyMatrix))]
        for row in range(len(occupancyMatrix)):
            for col in range(len(occupancyMatrix[row])):
                x = (col * self.resolution) + self.origin.position.x + self.resolution / 2.0
                y = (row * self.resolution) + self.origin.position.y + self.resolution / 2.0
                cost = occupancyMatrix[row][col]
                self.theMap[row][col] = Node(x, y, cost, row, col)  # step to initialize nodes

                self.waitForMap = False  # set variable that is referenced externally

    # A useful abstraction called by main. Tells the robot who's boss.
    # Contains almost all centroid functionality.
    def doYourJob(self):
        self.exploring = True
        self.findFrontierNodes(self.theMap)
        self.sortFrontier(self.theFrontier)

        self.frontierSize(self.sortedFrontier)  # returns data into "frontierToMove"

        if len(self.frontierToMove) != 0:  # Catches cases where there are no frontiers
            self.centroidFinder(self.frontierToMove)

        self.pubFrontier.publish(self.createGridCellsFromArray(self.theFrontier))  # Published to /frontier
        self.pubTheFrontier.publish(self.createGridCellsFromArray(self.frontierToMove))  # Published to /theFrontier

        if len(self.frontierToMove) < self.frontierThreshold:  # check if frontier is substantial enough to move to
            return False  # if less, return false and say "I can't do my job, sorry boss"
        else:
            return True  # else, accept orders and do its job.

            # Function checks if nodes are part of a frontier, returns array of them.

    def findFrontierNodes(self, map):

        self.theFrontier = []

        for row in range(len(map)):
            for col in range(len(map[row])):
                if map[row][col] != None:
                    # Check if a node is free and has undiscovered space next to it
                    if (map[row][col].cost == self.freeThreshold) and self.hasFrontier(map[row][col], row, col):
                        map[row][col].frontierNumber = 0  # Assign node to group of found nodes
                        self.theFrontier.append(map[row][col])

                        # sortFrontier is passed self.theFrontier, and returns a list of lists.
                        # The first list is indexed by [frontier number - 1]
                        # the second list is of the actual nodes that are part of the frontier group

    def sortFrontier(self, frontierNodes):
        frontierCounter = 1  # variable to keep track of groups
        self.sortedFrontier = []

        for each in range(len(frontierNodes)):
            if (frontierNodes[each].frontierNumber == 0):
                frontierGroup = []
                frontierNodes[each].frontierNumber = frontierCounter
                frontierGroup.append(frontierNodes[each])  # Add the home node to the list

                # Call recursive node finding function that no longer returns lists of lists of lists of..
                recurse = self.getNeighborFrontier(frontierNodes[each], frontierCounter)

                if recurse != None:  # if the list even exists, check if it has length
                    if len(recurse) > 0:
                        for numnum in range(len(recurse)):
                            frontierGroup.append(recurse[numnum])  # take out the data and add it to frontier group

                print(frontierCounter, len(frontierGroup))

                self.sortedFrontier.append(frontierGroup)  # add the current frontier group to the list of all groups

                frontierCounter += 1  # increment the groups

                # Gets neighbors of current node that is part of the frontier, returns a list of nodes
                # Function recurses until there are 0 nodes part of the frontier

    def getNeighborFrontier(self, node, frontierNumber):
        neighbors = []
        row = node.row
        col = node.col

        recursionIsHard = 0  # variable instantiated to remind me to think twice before
        # implementing recursion. Function recursed until out of memory
        # because value was hard coded wrong. Now, paramaterized.

        # Check the neighbors of the current node.
        if self.theMap[row + 1][col - 1].frontierNumber == recursionIsHard:
            self.theMap[row + 1][col - 1].frontierNumber = frontierNumber
            neighbors.append(self.theMap[row + 1][col - 1])

        if self.theMap[row + 1][col].frontierNumber == recursionIsHard:
            self.theMap[row + 1][col].frontierNumber = frontierNumber
            neighbors.append(self.theMap[row + 1][col])

        if self.theMap[row + 1][col + 1].frontierNumber == recursionIsHard:
            self.theMap[row + 1][col + 1].frontierNumber = frontierNumber
            neighbors.append(self.theMap[row + 1][col + 1])

        if self.theMap[row][col - 1].frontierNumber == recursionIsHard:
            self.theMap[row][col - 1].frontierNumber = frontierNumber
            neighbors.append(self.theMap[row][col - 1])

        if self.theMap[row][col + 1].frontierNumber == recursionIsHard:
            self.theMap[row][col + 1].frontierNumber = frontierNumber
            neighbors.append(self.theMap[row][col + 1])

        if self.theMap[row - 1][col + 1].frontierNumber == recursionIsHard:
            self.theMap[row - 1][col + 1].frontierNumber = frontierNumber
            neighbors.append(self.theMap[row - 1][col + 1])

        if self.theMap[row - 1][col].frontierNumber == recursionIsHard:
            self.theMap[row - 1][col].frontierNumber = frontierNumber
            neighbors.append(self.theMap[row - 1][col])

        if self.theMap[row - 1][col - 1].frontierNumber == recursionIsHard:
            self.theMap[row - 1][col - 1].frontierNumber = frontierNumber
            neighbors.append(self.theMap[row - 1][col - 1])

            # The actual recursive part.
        if len(neighbors) > 0:
            for each in range(len(neighbors)):

                recurse = self.getNeighborFrontier(neighbors[each], frontierNumber)  # recurse!!! 8^)

                if recurse != None:  # same catch statement as in above method, parses recursion data
                    if len(recurse) > 0:
                        for numnum in range(len(recurse)):
                            neighbors.append(recurse[numnum])

            return neighbors

    # Function checks if a node has an unexplored node next to it
    def hasFrontier(self, node, row, col):
        frontierLevel = 0  # how many frontier nodes are next to base node

        neighbors = self.getNeighbors(node, row, col)

        for each in range(len(neighbors)):
            if neighbors[each].cost == -1:
                frontierLevel += 1  # increment frontier level if found an unexplored node

        if frontierLevel > 0:  # logic of function, if frontier level greater than 0, is a frontier
            return True
        else:
            return False

    # Gets neighbors of current node, why even pass it node??
    # Honestly, looking back, this could have been multipurposed for that if statement a
    # couple methods above. Would've been better engineering.
    def getNeighbors(self, node, row, col):
        neighbors = []
        neighbors.append(self.theMap[row + 1][col - 1])
        neighbors.append(self.theMap[row + 1][col])
        neighbors.append(self.theMap[row + 1][col + 1])
        neighbors.append(self.theMap[row][col - 1])
        neighbors.append(self.theMap[row][col + 1])
        neighbors.append(self.theMap[row - 1][col + 1])
        neighbors.append(self.theMap[row - 1][col])
        neighbors.append(self.theMap[row - 1][col - 1])
        return neighbors

        # Given an array of nodes, returns grid cells message ready to be published

    def createGridCellsFromArray(self, nodeArray):
        grid = GridCells()
        grid.header.seq = 1
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = self.frameID
        grid.cell_width = self.resolution
        grid.cell_height = self.resolution
        for node in nodeArray:
            point = Point(node.x, node.y, 0)
            grid.cells.append(point)
        return grid

    def frontierSize(self, listOfNodeArrays):
        largestGroup = 0
        groupNumber = 0
        for eachList in range(len(listOfNodeArrays)):
            if len(listOfNodeArrays[eachList]) > largestGroup:  # check if length is greater than past largest
                largestGroup = len(listOfNodeArrays[eachList])  # if yes, set this group as the new largest
                groupNumber = listOfNodeArrays[eachList][0].frontierNumber

        print("largest group is", groupNumber)

        if len(listOfNodeArrays) != 0:
            self.frontierToMove = listOfNodeArrays[groupNumber - 1]

            return listOfNodeArrays[groupNumber - 1]  # remember 0 indexing!!!!

    # Given a node array, finds centroid and returns x, y
    def centroidFinder(self, nodeArray):
        self.pubTheFrontier.publish(self.createGridCellsFromArray(nodeArray))
        number = len(nodeArray)
        time = 0
        xValue = 0
        yValue = 0

        while time < number:  # thanks for this code brandon ahahaha
            xValue = xValue + nodeArray[time].x
            yValue = yValue + nodeArray[time].y
            time = time + 1

        xValue = xValue / number  # simply an averaging function
        yValue = yValue / number

        print(xValue, yValue)  # print out location of centroid in x,y coordinates

        self.goalx = xValue
        self.goaly = yValue

    # used for rotating the robot
    def rotate(self):
        global pub
        twist = Twist()
        angle = 2.05
        distance = 0.0
        while abs(angle - distance) > .1:
            # print distance
            if distance < angle:
                speed = 0.75
            if distance > angle:
                speed = -0.75
            distance = distance + speed * .1
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = speed
            self.spinPub.publish(twist)
            rospy.sleep(.1)

        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.spinPub.publish(twist)

    # honestly not the cheesiest thing in our code, used for finding if the robot has
    # navigated to the goal or not. Possible future corrections could be to actually read
    # the messages that are passed, and find out what the status says, then act accordingly
    def explorationStatus(self, message):
        self.exploring = False

        # use this function to send a navigation goal with properly formatted message

    def navToGoal(self):
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frameID

        goal.pose.position.x = self.goalx
        goal.pose.position.y = self.goaly
        goal.pose.position.z = 0

        # please, anything but quaternions
        goal.pose.orientation.x = 0
        goal.pose.orientation.y = 0
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.6

        self.navGoalPub.publish(goal)

    # simple function to let a process run, used for navigating to goals.
    def run(self):
        self.running = True
        print("started moving")
        while self.running:
            self.spinRate.sleep()
            self.running = self.exploring  # check if we have arrived at goal


# The program's primary executing section
if __name__ == '__main__':

    rospy.init_node('theBoys')
    node = FinalProject()  # initialize our program

    exploring = True
    moving = True

    while exploring:

        # please don't look at this part
        node.rotate()
        rospy.sleep(.5)
        node.rotate()
        node.rotate()
        rospy.sleep(.5)
        node.rotate()
        node.rotate()
        rospy.sleep(.5)
        node.rotate()
        node.rotate()
        rospy.sleep(.5)
        node.rotate()
        # thanks, okay now you can look again

        node.waitForMap = False
        while node.waitForMap:
            node.rotate()
            rospy.sleep(.5)
            print("waiting for a new map")

        exploring = node.doYourJob()  # where most of the magic happens

        if (exploring):
            node.navToGoal()
            node.run()
            print("navigated to goal")

        node = FinalProject()  # reinstantiate node, fresh start each time

    print("It's finally over, the last robot")  # we made it through the unified courses
