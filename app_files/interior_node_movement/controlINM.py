from subprocess import call

allNodesGroup = "allnodes"

#Get node ids ("0613066d", )
nodeIds = {"1":"06130622",}
nodeLocations = {"06130622":()}

groups = set()


def moveTo():
	print("-------------\nList of nodes to move: " + str(nodeIds))
	print("List of node locations: " + str(nodeLocations))
	chosen = str(input("Please enter the node to move: \n"))
	chosenStr = nodeIds[chosen]
	if nodeLocations[chosenStr] == tuple():
		call(["mosquitto_pub", "-h", "localhost", "-p", "1886", "-t", nodeIds[chosen], "-m", "9c00:00"]) # Assuming 0,0 starting location facing 0 degrees 
	chosenCoords = str(input("Please enter the coordinates for the target location as for example; 10:5 for (10,5): \n"))
	splitResult = chosenCoords.split(":")
	nodeLocations[nodeIds[chosen]] = (splitResult[0], splitResult[1])
	call(["mosquitto_pub", "-h", "localhost", "-p", "1886", "-t", nodeIds[chosen], "-m", "9t"+chosenCoords])
	print("Moved " + str(nodeIds[chosen]) + " to " + str(nodeLocations[nodeIds[chosen]]))

def joinAGroup():
	print("-------------\nList of nodes to join a group: " + str(nodeIds))
	chosenNode = str(input("Please enter the node to join a group: \n"))
	chosenGroup = str(input("Please enter the group for it to subscribe to: \n"))
	call(["mosquitto_pub", "-h", "localhost", "-p", "1886", "-t", nodeIds[chosenNode], "-m", "1"+chosenGroup])
	call(["mosquitto_pub", "-h", "localhost", "-p", "1886", "-t", nodeIds[chosenNode], "-m", "1"+chosenGroup])
	print("Sent sub command to " + chosenNode + ".\n")
	groups.add(chosenGroup)

# def swapGroup():
# 	print("-------------\nList of nodes to swap: " + str(nodeIds))
# 	chosen = str(input("Please enter the first node to move: \n"))
# 	chosen2 = str(input("Please enter the node to take the previous one's place: \n"))
# 	call(["mosquitto_pub", "-h", "localhost", "-p", "1886", "-t", nodeIds[chosen], "-m", "9c00:00"]) # Assuming 0,0 starting location facing 0 degrees 

# def groupMoveTo():
# 	print("-------------\nList of groups: " + str(groups))
# 	chosenGroup = str(input("Please enter the group to move: "))
# 	numberOfNodes = input("Please enter the number of nodes in the group: ")
# 	chosenX = str(input("Please enter the x coordinate for the target location as for example; 10 for (10,5): \n")) # assumes no decimal for this one
# 	chosenY = str(input("Please enter the y coordinate for the target location as for example; 10 for (10,5): \n")) # assumes no decimal for this one
# 	for i in range(numberOfNodes):
# 		tempCoords = "" + chosenX + "." + (i*0.20)  + ":" + chosenY + (i*0.20)
# 		call(["mosquitto_pub", "-h", "localhost", "-p", "1886", "-t", chosenGroup, "-m", "9t"+tempCoords]) # Assumes nodes already know where they are
# 		print("Sent target " + tempCoords +" to " + chosenGroup + "'s " + i + " node.\n")


	

def movementControl(nodeIds):
	while True:
		print("-------------\nList of nodes: " + str(nodeIds))
		chosen = str(input("(1) Join a group\n(2) Move node to target location\n(3) Move group to target location\n(Else) Exit movement control loop\nEnter selection: \n"))
		if chosen == "1":
			joinAGroup()
		elif chosen == "2":
			moveTo()
		elif chosen == "3":
			groupMoveTo()
		else:
			print("Exiting back to the main menu.\n")
			break;


print("---Welcome to the Interior Node Movement Script.---\n")


print("(default node IDs = " + str(nodeIds) + "\n")

nextInput = ""
while nextInput != "x":
	nextInput = str(input("Enter a node ID or 'x' to stop entering node IDs: \n"))
	if nextInput != "x":
		nodeIds[len(nodeIds)] = nextInput

while True: 
	y = str(input("To subscribe the nodes given to localization info enter (1), to give movement commands enter 2.\n"))
	if y == "1":
		for nodeIndex, node in nodeIds.items():
			#call(["mosquitto_pub", "-h", "localhost", "-p", "1886", "-t", node, "-m", "1"+allNodesGroup])
			call(["mosquitto_pub", "-h", "localhost", "-p", "1886", "-t", node, "-m", "1"+allNodesGroup])
			print("Sent sub command to " + node + ".\n")
	elif y == "2":
		movementControl(nodeIds)