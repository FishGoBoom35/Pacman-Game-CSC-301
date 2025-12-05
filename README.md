PacMan Program that attempts to use a BFS graphing algorithm for Blinky, a A* graphing algorithm for Pinky, a hybrid of the two for Inky, and Clyde still the same.
Original PacMan code from https://github.com/Teanlouise/Pacman-Game, essentially all of the code from this project is theirs, outside of the above mentioned Ghost algorithms, a slightly modified map, and slighty
modified code for speed of pacman and ghosts etc.

We used whatever resources we could, that being ChatGPT, GeeksForGeeks, StackOverFlow, etc. and whatever else we could find that would help.

Inspiration for PacMan and the algorithm use
Novikov, A., Yakovlev, S., & Gushchin, I. (2025). Exploring the possibilities of MADDPG for UAV swarm control by simulating in Pac-Man environment. Radioelectronic and Computer Systems, 2025(1), 327–337. 
https://doi.org/10.32620/reks.2025.1.21

Summary:
To be honest, the original program seemed pretty stable up until starting to really modify the code for what we wanted, that is when a lot of the problems started showing up and by that point it was way too close to the deadline to completely restart. In hindsight, should have looked earlier on and sooner. But with what is done, tried to change the format to be more like the original pacman, but that created a larger mess, so, decided to just focus more on the algorithms itself, although granted even that seems to be a shitshow. I can't figure out if certain things the ghosts do are because of what was implemented or if its because of the base code conflicting with the changes, it is hard to figure out what the base code does and all of the intricacies/weaknesses/strengths are for the base code (since I didnt write it, and there is more "TODO's" than I remember seeing, but despite all this, its been a huge learning experience. The largest weaknesses mostly has to do with an inability to source where some of the problems are located (maybe its what was changed conflicting with the base code). The strengths are, I think Inkys algorithm is pretty good, especially in culmination with blinky and pinky. The ghosts compliment each other pretty well, and I dont know how possible it would be for pacman if he didnt start a little faster. There is also no middle "teleport", since it was kind of late to implement and unfortunately the base pacman github dis not have that, maybe will add in the future. For weaknesses, the ghosts not being able to change directions I think is the biggest problem, but thats base pacman. The other main weakness is kind of based on what was just said. The engine evaluates ghost targets only once per movement update, meaning the ghosts do not always follow perfectly if Pac-Man changes direction rapidly, since once they move down a corridor they have to commit to that.

Blinky was implemented using a pure Breadth-First Search (BFS) approach. BFS guarantees the shortest path in an unweighted grid, allowing Blinky to aggressively pursue Pac-Man. Pinky was rewritten using an A* search algorithm aimed at a “ambush” tile four spaces ahead of Pac-Man. This gave Pinky a more "strategic" chase pattern compared to Blinky’s direct pursuit. Clyde’s behavior was rebuilt to match the classic design: when farther than eight tiles from Pac-Man, he uses BFS to chase him, but when he gets too close, his target immediately switches to the bottom-left corner, producing the characteristic “cowardly retreat.” Finally, Inky was designed as a hybrid ghost, comparing a BFS path directly to Pac-Man with an A* path toward a projected ambush point (found this to be a much better method than having the two algorithms rotate). Inky selects whichever path is shorter, allowing him to "predict" Pac-Man, and create a more lethal ghost.


Files that were changed from the original base code (Teanloiuses code):
BoardItem.java
BoardView.java
game2_big.map
GhostType.java
Launcher.java
MainView.java
MainViewModel.java
PacmanBoard.java
Blinky.java
Clyde.java
Ghost.java
Inky.java
PacmanGame.java
Phase.java
Pinky.java
Mainview.java
Mainviewmodel.java
The Readme.md obviously
