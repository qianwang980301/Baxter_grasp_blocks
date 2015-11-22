# Baxter_grasp_blocks

This repository is for the robotice final project

co-workers:
	Shipei Tian
	Zhiang Chen
	Alex DeFiore
	Qian Wang


Needed methods

void moveArmsBack()
//this method is used at the beginning to move Baxter's arms to a safe start position

boolean findTableTop()
//this method finds the height of the table and returns true if found, otherwise false

boolean checkForHand()
//this method checks if there is a hand above the tabletop

boolean isBlock()
//this method checks if there is a block on the table. If there is, store height, color, smaller width, and Pose

Pose getBlockPose()
//this method gets the Pose of the block stored in isBlock

Float64 getWidth()
//this method gets the smaller width of the block

Float64 getHeight()
//this method gets the height of the block

Eigenvector getColor()
//this method gets the color of the block, stored in an Eigenvector

Pose getCurrentPose()
//this method gets the current Pose of Baxter's arm

boolean planPath(Pose currentPose, Pose targetPose)
//this method plans and stores the movement path for Baxter from her current arm Pose to the target Pose. The plan should be stored in a way useable by movement methods. The method returns whether or not a path was successfully planned

boolean executePath()
//this method executes the path planned in planPath, returns true upon successful completion, otherwise false

boolean grabBlock(Float64 width)
//this method closes the grippers enough to grab a block of the given width

boolean planRedMovement(Pose current)
//this method plans movement of the red block from the current pose

boolean planBlueMovement(Pose current)

boolean planWhiteMovement(Pose current)

boolean planBlackMovement(Pose current)

boolean planWoodMovement(Pose current)

boolean planGreenMovement(Pose current)

boolean releaseBlock()
//this method extends the grippers to their maximum width to drop the block and returns true upon completion



General note
All planning functions should store their plan in some variable accessible by the executePath function. Then execute path can be reused for all movements of Baxter
