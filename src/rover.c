#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "bitmap.h"

#define MAP_SIZE 128
#define MAX_GOALS 100

// Structure to hold map data
typedef struct {
    unsigned char rover_x, rover_y;
    unsigned char topography[MAP_SIZE][MAP_SIZE];
    unsigned char terrain_type[MAP_SIZE][MAP_SIZE];
    bool is_rover[MAP_SIZE][MAP_SIZE];
    bool is_goal[MAP_SIZE][MAP_SIZE];
    bool is_corrupted[MAP_SIZE][MAP_SIZE];
    int ngoals;
    int goals_x[MAX_GOALS];
    int goals_y[MAX_GOALS];
} MapData;

// Function prototypes
bool check_parity(unsigned char byte);
void decode_map_data(const char* filename, MapData* map);
void apply_3x3_filter(MapData* map);
bool is_feasible_path(MapData* map, int start_x, int start_y, int end_x, int end_y);
int calculate_energy(MapData* map, int start_x, int start_y, int end_x, int end_y);
void simulate_path(MapData* map, int* final_x, int* final_y, int* energy, bool* feasible);

// Check parity bit for error detection
bool check_parity(unsigned char byte) {
    int count = 0;
    for (int i = 0; i < 7; i++) {
        if (byte & (1 << i)) count++;
    }
    return (count % 2) == (byte & 0x80 ? 1 : 0);
}

// Decode binary map data
void decode_map_data(const char* filename, MapData* map) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        printf("Input error: unable to process data/commands.\n");
        exit(1);
    }
    
    // Read rover position
    fread(&map->rover_x, 1, 1, file);
    fread(&map->rover_y, 1, 1, file);
    
    // Initialize arrays
    memset(map->is_rover, false, sizeof(map->is_rover));
    memset(map->is_goal, false, sizeof(map->is_goal));
    memset(map->is_corrupted, false, sizeof(map->is_corrupted));
    map->ngoals = 0;
    
    // Read map data row by row
    for (int y = 0; y < MAP_SIZE; y++) {
        for (int x = 0; x < MAP_SIZE; x++) {
            unsigned char byte;
            fread(&byte, 1, 1, file);
            
            // Check for corruption
            if (!check_parity(byte)) {
                map->is_corrupted[y][x] = true;
                continue;
            }
            
            // Extract data from byte
            unsigned char height = byte & 0x07;           // bits 0-2
            unsigned char terrain = (byte >> 3) & 0x03;   // bits 3-4
            bool is_goal_cell = (byte >> 5) & 0x01;       // bit 5
            bool is_rover_cell = (byte >> 6) & 0x01;      // bit 6
            
            map->topography[y][x] = height;
            map->terrain_type[y][x] = terrain;
            map->is_goal[y][x] = is_goal_cell;
            map->is_rover[y][x] = is_rover_cell;
            
            // Collect science goals
            if (is_goal_cell && map->ngoals < MAX_GOALS) {
                map->goals_x[map->ngoals] = x;
                map->goals_y[map->ngoals] = y;
                map->ngoals++;
            }
        }
    }
    
    fclose(file);
    
    // Apply 3x3 filter to corrupted cells
    apply_3x3_filter(map);
}

// Apply 3x3 mode filter to corrupted cells
void apply_3x3_filter(MapData* map) {
    for (int y = 0; y < MAP_SIZE; y++) {
        for (int x = 0; x < MAP_SIZE; x++) {
            if (map->is_corrupted[y][x]) {
                // Collect valid neighbors
                int height_counts[8] = {0};
                int terrain_counts[4] = {0};
                
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int ny = y + dy;
                        int nx = x + dx;
                        
                        if (ny >= 0 && ny < MAP_SIZE && nx >= 0 && nx < MAP_SIZE && 
                            !map->is_corrupted[ny][nx]) {
                            height_counts[map->topography[ny][nx]]++;
                            terrain_counts[map->terrain_type[ny][nx]]++;
                        }
                    }
                }
                
                // Find mode for height
                int max_height_count = 0;
                int mode_height = 0;
                for (int i = 0; i < 8; i++) {
                    if (height_counts[i] > max_height_count) {
                        max_height_count = height_counts[i];
                        mode_height = i;
                    } else if (height_counts[i] == max_height_count && i < mode_height) {
                        mode_height = i;
                    }
                }
                
                // Find mode for terrain
                int max_terrain_count = 0;
                int mode_terrain = 0;
                for (int i = 0; i < 4; i++) {
                    if (terrain_counts[i] > max_terrain_count) {
                        max_terrain_count = terrain_counts[i];
                        mode_terrain = i;
                    } else if (terrain_counts[i] == max_terrain_count && i < mode_terrain) {
                        mode_terrain = i;
                    }
                }
                
                map->topography[y][x] = mode_height;
                map->terrain_type[y][x] = mode_terrain;
                map->is_corrupted[y][x] = false;
            }
        }
    }
}

// Check if path is feasible using BFS
bool is_feasible_path(MapData* map, int start_x, int start_y, int end_x, int end_y) {
    bool visited[MAP_SIZE][MAP_SIZE];
    memset(visited, false, sizeof(visited));
    
    // Queue for BFS
    int queue_x[MAP_SIZE * MAP_SIZE];
    int queue_y[MAP_SIZE * MAP_SIZE];
    int front = 0, rear = 0;
    
    queue_x[rear] = start_x;
    queue_y[rear] = start_y;
    rear++;
    visited[start_y][start_x] = true;
    
    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};
    
    while (front < rear) {
        int x = queue_x[front];
        int y = queue_y[front];
        front++;
        
        if (x == end_x && y == end_y) {
            return true;
        }
        
        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            
            if (nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE && !visited[ny][nx]) {
                // Check if terrain is traversable
                if (map->terrain_type[ny][nx] == 3) continue;
                
                // Check slope constraint
                int slope = abs(map->topography[y][x] - map->topography[ny][nx]);
                if (slope > 1) continue;
                
                visited[ny][nx] = true;
                queue_x[rear] = nx;
                queue_y[rear] = ny;
                rear++;
            }
        }
    }
    
    return false;
}

// Calculate energy required for path
int calculate_energy(MapData* map, int start_x, int start_y, int end_x, int end_y) {
    // This is a simplified version - in practice you'd need to trace the actual path
    // For now, return a basic calculation
    int energy = 0;
    int x = start_x, y = start_y;
    
    // Simple path calculation (this would need to be more sophisticated)
    while (x != end_x || y != end_y) {
        // Terrain energy
        switch (map->terrain_type[y][x]) {
            case 0: energy += 1; break;
            case 1: energy += 2; break;
            case 2: energy += 4; break;
        }
        
        // Slope energy
        if (x < end_x) x++;
        else if (x > end_x) x--;
        if (y < end_y) y++;
        else if (y > end_y) y--;
        
        if (x != end_x || y != end_y) {
            int slope = abs(map->topography[y][x] - map->topography[y][x]);
            energy += slope * 10;
        }
    }
    
    return energy;
}

// Simulate rover path
void simulate_path(MapData* map, int* final_x, int* final_y, int* energy, bool* feasible) {
    int x = map->rover_x;
    int y = map->rover_y;
    int heading = 0; // 0=North, 1=East, 2=South, 3=West
    *energy = 0;
    *feasible = true;
    
    char command[20];
    int distance;
    
    while (scanf("%s", command) == 1) {
        if (strcmp(command, "end") == 0) break;
        
        if (strcmp(command, "forward") == 0) {
            scanf("%d", &distance);
            for (int i = 0; i < distance; i++) {
                int new_x = x, new_y = y;
                switch (heading) {
                    case 0: new_y++; break; // North
                    case 1: new_x++; break; // East
                    case 2: new_y--; break; // South
                    case 3: new_x--; break; // West
                }
                
                // Check bounds
                if (new_x < 0 || new_x >= MAP_SIZE || new_y < 0 || new_y >= MAP_SIZE) {
                    *feasible = false;
                    *final_x = x;
                    *final_y = y;
                    return;
                }
                
                // Check terrain type
                if (map->terrain_type[new_y][new_x] == 3) {
                    *feasible = false;
                    *final_x = x;
                    *final_y = y;
                    return;
                }
                
                // Check slope
                int slope = abs(map->topography[y][x] - map->topography[new_y][new_x]);
                if (slope > 1) {
                    *feasible = false;
                    *final_x = x;
                    *final_y = y;
                    return;
                }
                
                // Add energy costs
                switch (map->terrain_type[new_y][new_x]) {
                    case 0: *energy += 1; break;
                    case 1: *energy += 2; break;
                    case 2: *energy += 4; break;
                }
                *energy += slope * 10;
                
                x = new_x;
                y = new_y;
            }
        }
        else if (strcmp(command, "backwards") == 0) {
            scanf("%d", &distance);
            for (int i = 0; i < distance; i++) {
                int new_x = x, new_y = y;
                switch (heading) {
                    case 0: new_y--; break; // South
                    case 1: new_x--; break; // West
                    case 2: new_y++; break; // North
                    case 3: new_x++; break; // East
                }
                
                // Check bounds
                if (new_x < 0 || new_x >= MAP_SIZE || new_y < 0 || new_y >= MAP_SIZE) {
                    *feasible = false;
                    *final_x = x;
                    *final_y = y;
                    return;
                }
                
                // Check terrain type
                if (map->terrain_type[new_y][new_x] == 3) {
                    *feasible = false;
                    *final_x = x;
                    *final_y = y;
                    return;
                }
                
                // Check slope
                int slope = abs(map->topography[y][x] - map->topography[new_y][new_x]);
                if (slope > 1) {
                    *feasible = false;
                    *final_x = x;
                    *final_y = y;
                    return;
                }
                
                // Add energy costs
                switch (map->terrain_type[new_y][new_x]) {
                    case 0: *energy += 1; break;
                    case 1: *energy += 2; break;
                    case 2: *energy += 4; break;
                }
                *energy += slope * 10;
                
                x = new_x;
                y = new_y;
            }
        }
        else if (strcmp(command, "left") == 0) {
            scanf("%d", &distance);
            heading = (heading - distance/90 + 4) % 4;
        }
        else if (strcmp(command, "right") == 0) {
            scanf("%d", &distance);
            heading = (heading + distance/90) % 4;
        }
    }
    
    *final_x = x;
    *final_y = y;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        printf("Input error: unable to process data/commands.\n");
        return 1;
    }
    
    char mode = argv[1][0];
    char* mapfile = argv[2];
    
    MapData map;
    decode_map_data(mapfile, &map);
    
    if (mode == 'm') {
        if (argc != 5) {
            printf("Input error: unable to process data/commands.\n");
            return 1;
        }
        
        int x = atoi(argv[3]);
        int y = atoi(argv[4]);
        
        if (x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE) {
            printf("Input error: unable to process data/commands.\n");
            return 1;
        }
        
        printf("Grid cell (x=%d, y=%d)\n", x, y);
        printf("Topo height: %dm\n", map.topography[y][x]);
        printf("Terrain type: %d\n", map.terrain_type[y][x]);
        
        if (map.is_goal[y][x]) {
            printf("Specified location is a science goal\n");
        }
        if (map.is_rover[y][x]) {
            printf("Specified location is initial rover position\n");
        }
    }
    else if (mode == 'c') {
        int final_x, final_y, energy;
        bool feasible;
        
        simulate_path(&map, &final_x, &final_y, &energy, &feasible);
        
        if (feasible) {
            printf("Path is feasible\n");
            printf("Energy required to complete path: %d\n", energy);
        } else {
            printf("Path is not feasible\n");
            printf("Last feasible position: (x=%d, y=%d)\n", final_x, final_y);
        }
    }
    else if (mode == 'f') {
        if (argc != 4) {
            printf("Input error: unable to process data/commands.\n");
            return 1;
        }
        
        int goal_index = atoi(argv[3]);
        
        if (goal_index < 0 || goal_index >= map.ngoals) {
            printf("Input error: unable to process data/commands.\n");
            return 1;
        }
        
        int goal_x = map.goals_x[goal_index];
        int goal_y = map.goals_y[goal_index];
        
        if (is_feasible_path(&map, map.rover_x, map.rover_y, goal_x, goal_y)) {
            printf("Feasible path exists.\n");
        } else {
            printf("There are no feasible paths to this location.\n");
        }
    }
    else {
        printf("Input error: unable to process data/commands.\n");
        return 1;
    }
    
    return 0;
}
