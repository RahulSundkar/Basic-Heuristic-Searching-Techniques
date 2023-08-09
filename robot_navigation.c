// This code demonstrates the Best First Search and A* algorithms
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

typedef struct{
    int x; // X co-ordinate
    int y; // Y co-ordinate
}Coord;

// Structure for a node in Traversal Graph
typedef struct Node{
    int next_size; // Size of Array containing branches (Node** next)
    struct Node** next; // Array of branches
    struct Node* prev; // Previous branch in path for backtracking
    Coord pos; // Position of Robot in Grid
} Node;

// Calculate Manhattan Distance between two Co-ordinates
int calc_dist(Coord a, Coord b){
    int dist = abs(a.x - b.x) + abs(a.y - b.y);
    return dist;
}

// Add Node to open list 
void add_open(Node* c, Node*** op_list, int* current_size){ // Takes pointer to open list to make changes in it
    (*current_size)++;
    // wait till size variable catches up with size at declaration
    if(sizeof(*op_list) / sizeof(Node*) < *current_size)
    {
        *op_list = (Node**) realloc(*op_list, *current_size * sizeof(Node*));
    }
    (*op_list)[*current_size - 1] = c;
}

// Return index of element in open list with best heuristic value(shortest distance from goal) 
int check_open(Coord start, Coord goal, Node** op_list, int size, int algo){
    if(size > 0) // Check if Open list has elements in it
    {
        int index_of_min = 0;
        int min_dist = INT_MAX;

        if(algo == 0)
        {
            for(int i = 0; i < size; i++)
            {
                Coord current = op_list[i]->pos;
                int temp_dist = calc_dist(current, goal);
                if(temp_dist < min_dist)
                {
                    min_dist = temp_dist;
                    index_of_min = i;
                }
            }
        }
        else if(algo == 1){
            for(int i = 0; i < size; i++)
            {
                Coord current = op_list[i]->pos;
                int temp_dist = calc_dist(current, start) + calc_dist(current, goal);
                if(temp_dist < min_dist)
                {
                    min_dist = temp_dist;
                    index_of_min = i;
                }
            }
        }
        
        return index_of_min;
    }
    else{
        return -1;
    }
}

// Remove element from open list, extract its Coord member and add that to closed list 
void rem_open_add_closed(int position, Node*** op_list, int* op_size, Coord** cl_list, int* cl_size){
    Coord c = (*op_list)[position]->pos; // Stores pos of Element to be removed
    for(int i = position; i < *op_size - 1; i++)
    {
        (*op_list)[i] = (*op_list)[i + 1];
    }

    (*op_size)--;
    // wait till size variable catches up with size at declaration
    if(sizeof(*op_list) / sizeof(Node*) < *op_size)
    {
        *op_list = (Node**) realloc(*op_list, *op_size * sizeof(Node*));
    }

    (*cl_size)++;
    // wait till size variable catches up with size at declaration
    if(sizeof(*cl_list) / sizeof(Coord*) < *cl_size)
    {
        *cl_list = (Coord*) realloc(*cl_list, *cl_size * sizeof(Coord));
    }
    (*cl_list)[*cl_size - 1] = c; // Add Coord to closed list
}

// Check if Co-ordinates are already present in the closed list 
int is_closed(int x, int y, Coord* cl_list, int size){
    for(int i = 0; i < size; i++)
    {
        if(x == cl_list[i].x && y == cl_list[i].y)
        {
            return 1;
        }
    }
    return 0;
}

// Check if Co-ordinates are present in obstacles list
int is_obstacle(int x, int y, Coord* obs_list, int size){
    for(int i = 0; i < size; i++)
    {
        if(x == obs_list[i].x && y == obs_list[i].y)
        {
            return 1;
        }
    }
    return 0;
}

// Find possible progressions for current node
void find_possibilities(Node** current, Coord* cl_list, int cl_size, Coord* obs_list, int obs_size, int grid_x, int grid_y){

    Coord neighbor_list[4]; // Define a list of neighbors
    Coord cur_pos = (*current)->pos; // Position of current Node in grid
    int x = cur_pos.x;
    int y = cur_pos.y;

    // Left
    neighbor_list[0].x = x-1; 
    neighbor_list[0].y = y; 

    // Right
    neighbor_list[1].x = x+1; 
    neighbor_list[1].y = y; 

    // Top
    neighbor_list[2].x = x; 
    neighbor_list[2].y = y-1; 

    // Bottom
    neighbor_list[3].x = x; 
    neighbor_list[3].y = y+1; 

    for(int i = 0; i < 4; i++)
    {
        int n_x = neighbor_list[i].x;
        int n_y = neighbor_list[i].y;

        if((n_x <= grid_x && n_y <= grid_y) && (n_x >= 1 && n_y >= 1) && !is_obstacle(n_x, n_y, obs_list, obs_size) && !is_closed(n_x, n_y, cl_list, cl_size)) // Checks if neighbor is valid i.e is present inside the grid, is not an obstacle or closed
        {
            Node* newnode = (Node*) malloc(sizeof(Node)); // Create Node* of the neighbor as a branch
            newnode->next_size = 0;
            newnode->next = (Node**) malloc (sizeof(Node*));
            newnode->pos.x = n_x;
            newnode->pos.y = n_y;
            newnode->prev = (*current); // For backtracking to get path

            (*current)->next_size += 1;
            int size = (*current)->next_size;
            // wait till size variable catches up with size at declaration
            if(sizeof((*current)->next) / sizeof(Node*) < size)
            {
                (*current)->next = (Node**) realloc((*current)->next, size * sizeof(Node*));
            }
            (*current)->next[size-1] = newnode; // Add branch
        }
    }

    // if((*current)->next_size > 0)
    // {
    //     return 1;
    // }
    // else{
    //     return 0;
    // }
}

void main(){
    printf("\nRobot Navigation Problem\n");
    printf("____________________________________________________________\n");
    printf("\nEnter 0 for BEST FIRST SEARCH\nEnter 1 for A* ALGORITHM\n");
    int algo;
    do{
        printf("Enter Choice: ");
        scanf("%d", &algo);
    }while(algo != 0 && algo != 1);

    // Dimensions of Grid
    int x_dim = 0, y_dim = 0;
    printf("\nEnter dimensions of grid \n");
    
    do{
        printf("Enter X axis: ");
        scanf("%d", &x_dim);
    }while(x_dim < 1);
    
    do{
        printf("Enter Y axis: ");
        scanf("%d", &y_dim);
    }while(y_dim < 1);

    // Defining obstacles
    int num_obs = 0;
    do{
        printf("\nEnter number of obstacles: ");
        scanf("%d", &num_obs);
    }while(num_obs > (x_dim * y_dim));

    Coord* obstacle = (Coord*) malloc(num_obs * sizeof(Coord));

    for(int i = 0; i < num_obs; i++)
    {
        do{
            printf("Enter coordinates for #%d obstacle: \n", i+1);
            printf("X coord: ");
            scanf("%d", &obstacle[i].x);
            printf("Y coord: ");
            scanf("%d", &obstacle[i].y);
        }while(obstacle[i].x < 1 || obstacle[i].x > x_dim || obstacle[i].y < 1 || obstacle[i].y > y_dim);
    }

    // Start position
    Coord start;
    do{
        printf("\nEnter Start position: \n");
        printf("X coord: ");
        scanf("%d", &start.x);
        printf("Y coord: ");
        scanf("%d", &start.y);
    }while(start.x < 1 || start.x > x_dim || start.y < 1 || start.y > y_dim);

    // Goal position
    Coord goal;
    do{
        printf("\nEnter Goal position: \n");
        printf("X coord: ");
        scanf("%d", &goal.x);
        printf("Y coord: ");
        scanf("%d", &goal.y);
    }while(goal.x < 1 || goal.x > x_dim || goal.y < 1 || goal.y > y_dim);

    // Defining Open List:
    //      It is a list of Node*. 
    //      Elements point to Nodes in Traversal graph
    //      After we find the best heuristic from the open list we can directly jump to that open node in the Traversal graph
    int open_size = 0;
    Node** open_list = (Node**) malloc(5 * sizeof(Node*));

    // Defining Closed List:
    //      It is a list of Coord
    //      Stores nodes that have been explored
    int closed_size = 0;
    Coord* closed_list = (Coord*) malloc(5 * sizeof(Coord));

    // Create Root node of Traversal Graph
    Node* start_node = (Node*) malloc (sizeof(Node));
    start_node->next_size = 0;
    start_node->next = (Node**) malloc (sizeof(Node*));
    start_node->pos = start;
    start_node->prev = NULL; // No node before root

    // Add root to open list
    add_open(start_node, &open_list, &open_size);

    int open_index = 0; // Stores return value of check_open()
    Node* current = (Node*) malloc(sizeof(Node)); // Stores Node* from open_list at index open_index
    int success = 0; // Condition variable

    while(success != 1){
        // TEST
        // printf("Open List: ");
        // for(int i = 0; i < open_size; i++)
        // {
        //     printf("(%d, %d) ", open_list[i]->pos.x, open_list[i]->pos.y);
        // }
        // printf("\n");
        // printf("Closed List: ");
        // for(int i = 0; i < closed_size; i++)
        // {
        //     printf("(%d, %d) ", closed_list[i].x, closed_list[i].y);
        // }
        // printf("\n");
        // --

        open_index = check_open(start, goal, open_list, open_size, algo); // Get index of open node with the best heuristic value

        if(open_index >= 0){
            current = open_list[open_index];
        }
        else{ // If open list is empty, no path exists
            break;
        }

        printf("(%d, %d)\n", current->pos.x, current->pos.y);
        
        rem_open_add_closed(open_index, &open_list, &open_size, &closed_list, &closed_size); // Remove current from open list

        if(current->pos.x == goal.x && current->pos.y == goal.y) // Check if current is  at goal
        {
            success = 1;
            printf("\nGoal Reached!!!\n\n");
            break;
        }

        find_possibilities(&current, closed_list, closed_size, obstacle, num_obs, x_dim, y_dim); // Expand tree at current

        int num_branches = current->next_size;
        for(int i = 0; i < num_branches; i++)
        {
            Node* branch = current->next[i]; // Temp variable for branches
            add_open(branch, &open_list, &open_size); // Add branches to open list
        }
        
        // if(if_paths_found)
        // {
        //     int num_branches = current->next_size;
        //     for(int i = 0; i < num_branches; i++)
        //     {
        //         Node* branch = current->next[i];
        //         add_open(&branch, &open_list, &open_size);
        //     }
        //     rem_open_add_closed(open_index, &open_list, &open_size, &closed_list, &closed_size);
        // }
        // else{
        //     continue;
        // }
    };

    // Print Open List
    printf("Open List: ");
    for(int i = 0; i < open_size; i++)
    {
        printf("(%d, %d) ", open_list[i]->pos.x, open_list[i]->pos.y);
    }
    printf("\n");

    // Print Closed List
    printf("Closed List: ");
    for(int i = 0; i < closed_size; i++)
    {
        printf("(%d, %d) ", closed_list[i].x, closed_list[i].y);
    }
    printf("\n\n");

    if(success == 1)
    {
        int path_size = 0;
        Coord* rev_path = (Coord*) malloc (path_size * sizeof(Coord)); // List of Coord which stores the path in reverse

        while(current != NULL)
        {
            path_size += 1;
            rev_path = (Coord*) realloc (rev_path ,path_size * sizeof(Coord));
            rev_path[path_size-1] = current->pos; // Add Coord in reverse i.e. from Goal to Start
            current = current->prev;
        }

        // Print Path found by Best First Search 
        printf("Path is:\n");
        for(int i = path_size-1; i >= 0; i--) // Print path in reverse
        {
            int x = rev_path[i].x;
            int y = rev_path[i].y;

            printf("(%d, %d)", x,y);
            if(i != 0)
            {
                printf(" -> ");
            }
        }
    }
    else{
        printf("\nNO POSSIBLE PATH!!!\n\n");
    }
}   