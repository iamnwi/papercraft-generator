# Assignment 4: Final Project

1. maximal spanning forest for flattening
    1. prime algorithm
2. regular grid spatial data structure for overlap checking
    1. overlap checking
        1. vertices inside other triangle mesh
            1. the new flatted vertex is inside a flatted mesh?
            2. any flatted vertex is inside the new flatted triangle mesh?
        2. line intersections
            1. the two lines created by adding the new flatted vertex intersects any other lines of a flatted mesh?
    2. get near meshes from regular grid
        1. choose the max rectangle of the triangle mesh when looking up the associated grid cell
3. islands layout on paper
    1. rectangle placement problem
        a simple method:
            set paper width to the max width of islands, arrange islands line by line without space optimization
4. split into two sub-windows
    1. two view matrix
5. customized flattening area
    1. support non-contiguous selection areas
6. svg export
7. animation
    1. restore the related position of every vertex by traversing the MST
    2. every tree node of MST store the rotation quaternion, use it to construct rotation matrix
    3. compute the rotation matrix of every frame by interpolating rotation matrix
    4. the move of a subtree root should spread to all nodes of the subtree