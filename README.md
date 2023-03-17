# CSE 390 - HW2 - Undergraduate
> Contributors:\
> James Leonardi - <james.leonardi@stonybrook.edu>\
> Kevin Tao - <kevin.tao@stonybrook.edu>

## Solution Approach

### Algorithm
#### probably rewrite later.
The algorithm dynamically generates a map of the house as it navigates through it, storing it as a kind of graph.
Data structures:
- unordered_set **mapped**: stores the coordinates of all the mapped tiles
- unordered_set **visited**: stores the coordinates of all the mapped tiles that the robot has already visited.
- unordered_map **returnPath**: stores the most efficient path to return to the starting point.
- class **Node**: models a position in the house. Stores its coords, its neighbours, and its parent. Note that the coords for the node is not necessarily unique.
- struct **Position**: simply a struct of the x and y coordinates of a position.
- class **PositionHasher**: a custom hash function for the Position struct. This is used to store the Position struct in an unordered_set/unordered_map.

Upon the first invocation of the algorithm, the static variables will be set to their initial values. Most importantly, the start is set to a position of (0,0). Since we do not know the starting position of the robot, we put it at (0,0). We do not know where the robot actually starts. This limitation means we cannot keep a vector\<vector\> of the house, as negative indexing will become a major issue. Instead, the house is modeled as a graph using **Node**s and a set of coordinates.

There are some checks that happen before the bulk of the algorithm, pertaining to returning to the charger and staying to clean dirt, these will be outlined later.

The algorithm is as follows:
Create a **choice** vector that keeps track of the valid moves. For every direction (NESW), we'll check if there's a wall there. If there is, we just disregard that direction. Otherwise, we'll add it to the **choice** vector, add it to the neighbours of the current node, and insert it into the **mapped** set. Next, we need to do some stuff to properly manage the shortest path home. 
- We get the shortest path of the position in that direction, stored in **returnPath**.
- If that path is empty or if that path is longer than the current path, we update that path to the current path + the direction to get here.
- Otherwise, if the current path is empty or the path is longer than the direction's path, we update this path to the directions path + the direction to get there.

This procedure acts as a way to roughly minimize the shortest path home. I'm pretty sure it's not perfect, as there are probably some situations where it wouldn't produce the shortest one, but it does alleviate the issue greatly for the most part, particularly for snaking and winding paths.

Next, we need to generate a list of directions that get priority. This priority is given to positions that have not yet been visited. This is why the **visited** set is necessary. 

If there are no possible directions we can move, and the path is empty, we simply stay still. This will only happen if the charger is blocked with walls on all sides. If there are no priority directions, as in all directions are visited, we start backtracking through the **path** stack (technically a vector, but acts like a stack).

Otherwise, if there *is* a priority, we go in that direction. We mark that position as visited, and "move" to that path, by returning the direction and setting the current node to the node in that direction. We also append this move to the path stack, for later backtracking.

### Return Logic

The first thing we do in the algorithm at every step is check for an out of battery condition. This is done by fetching the returnPath for the current position, and checking that we wouldn't run out of battery by moving away.

Here, **returnQ** is used to keep a queue of directions to get back to the charger. We only set this once right when we start to return. As the robot makes its way back to the charger, we pop off the directions from the queue and return them. During this, we append the opposite direction to the **resumePath**, which keeps track of how to get back to where the robot was before it started returning. 

Once the robot reaches the charger, it will charge until it is full. After this is done, it will begin consuming the resumePath to get back to where it was previously.

Throughout this procedure we need the variable **curPos** which keeps track of the current position of the robot. This is usually an unnecessary redundancy, except for when the robot is returning to the charger. Because of the way the Node's neighbours and parents are populated, it makes it rather difficult to keep track of the current position when moving in a way that doesn't conform to the main algorithm. You might think that we can just rely on the returnPath always being correct, so that we can just keep track of the size of the returnPath to know when we are the charger. However, this is problematic due to how the returnQ is initially set. Maybe there is a workaround to this, but I found using the curPos variable to be the easiest solution.

Another way to doing this would be to just get the back() of the returnPath[c->coords], but this doesn't quite work. This is because we would need to keep track of Node c for when we resume. We wouldn't want to potentially end up with a screwed up c from mapping. Technically it's probably doable, by navigating through the neighbours of the Nodes, but this creates a lot of extra work and is probably not worth it.

#### There are likely a bunch of circular dependencies with shared_ptr in the code. This is due to originally using allocations with "new" and just replacing them. We maybe need to fix this later, but it shouldn't create any tangible issues for reasonable input sizes. Probably should use weak_ptr for neighbours/parent or something.



### Probably condense all this later for the final version.