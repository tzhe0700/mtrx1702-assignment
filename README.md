# MTRX1702 Mars Rover Assignment

## Overview
This project implements the software for a Mars planetary rover that performs daily exploration and science activities. The rover receives daily satellite imagery maps and uses this data to evaluate potential paths for navigation.

## Features Implemented

### 1. Binary Map Data Decoding
- Decodes satellite-communicated binary map data
- Implements parity bit checking for error detection
- Extracts topographic height, terrain type, science goals, and rover position
- Handles data corruption with 3x3 mode filtering

### 2. Path Evaluation
- Simulates rover movement based on command sequences
- Checks path feasibility considering terrain type and slope constraints
- Calculates energy requirements for traversing paths
- Handles rover orientation and movement commands (forward, backwards, left, right)

### 3. Feasible Path Detection
- Uses Breadth-First Search (BFS) to determine if feasible paths exist
- Considers terrain traversability and slope constraints
- Finds paths from rover start position to science goals

### 4. Command Line Interface
The program supports three modes:

#### Map Query Mode (m)
```
./rover m <mapfilename> <x> <y>
```
Displays information about a specific grid cell.

#### Path Check Mode (c)
```
./rover c <mapfilename>
```
Reads path commands from stdin and evaluates feasibility and energy requirements.

#### Check Feasible Mode (f)
```
./rover f <mapfilename> <i>
```
Determines if a feasible path exists to a specific science goal.

## Building and Running

### Build
```bash
make clean && make
```

### Run Examples
```bash
# Map query mode
./rover m maps/map001.bin 5 30

# Path check mode
./rover c maps/map001.bin
# Then input path commands via stdin

# Check feasible mode
./rover f maps/map001.bin 0
```

## Implementation Details

### Data Structures
- `MapData`: Comprehensive structure storing all map information
- Efficient 2D arrays for topography, terrain type, and metadata
- Dynamic goal tracking with arrays for science goal positions

### Error Handling
- Parity bit validation for data integrity
- 3x3 mode filtering for corrupted cells
- Input validation and bounds checking
- Comprehensive error messages for invalid inputs

### Algorithms
- BFS for path feasibility checking
- Mode calculation for corrupted data reconstruction
- Energy calculation based on terrain type and slope

## Files
- `src/rover.c`: Main implementation
- `src/bitmap.c`: Bitmap visualization utilities
- `include/bitmap.h`: Header file for bitmap functions
- `Makefile`: Build configuration
- `logbook.txt`: Development logbook
- `maps/`: Directory containing map binary files

## Development Log
See `logbook.txt` for detailed development process and design decisions.
