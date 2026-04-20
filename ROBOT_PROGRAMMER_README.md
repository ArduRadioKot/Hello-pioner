# Robot Programmer - System for Programming Robot Movements

## Overview
The Robot Programmer is a web-based interface that allows you to create and execute programs for your robot using a visual programming approach. You can set start and target points on a graph, add lift actions, and save/load programs.

## Features

### 1. Visual Point Selection
- **Set Start Point**: Click to select the robot's starting position on the graph
- **Set Target Point**: Click to select where the robot should move
- **Clear Selection**: Reset all selected points

### 2. Program Building
- **Add Move Action**: Add movement to the selected target node
- **Add Lift Up**: Add lift mechanism up action
- **Add Lift Down**: Add lift mechanism down action
- **Remove Actions**: Remove individual actions from the program

### 3. Program Management
- **Save Program**: Save the current program with a custom name
- **Execute Program**: Run the current program on the robot
- **Load Programs**: Load previously saved programs
- **Clear Program**: Clear the current program

### 4. Direct Lift Control
- **Lift Up**: Immediately lift the mechanism
- **Lift Down**: Immediately lower the mechanism

## How to Use

### Step 1: Access the Robot Programmer
1. Start the service: `python app.py`
2. Open your browser and go to `http://localhost:5002`
3. Click the "Robot Programmer" button

### Step 2: Set Up Points
1. Click "Set Start Point"
2. Click on a node in the graph to set the starting position (green node)
3. Click "Set Target Point" 
4. Click on another node to set the target position (red node)

### Step 3: Build Your Program
1. Click "Add Move Action" to add movement to the target node
2. Click "Add Lift Up" or "Add Lift Down" to add lift actions
3. Arrange actions in the desired order
4. Remove unwanted actions with the "Remove" button

### Step 4: Save and Execute
1. Click "Save Program" and enter a name
2. Click "Execute Program" to run it on the robot
3. Or load a saved program from the list and click "Run"

## API Endpoints

### Robot Programs
- `GET /api/robot/programs` - Get all saved programs
- `POST /api/robot/programs` - Save a new program
- `POST /api/robot/execute-program` - Execute a saved program

### Lift Control
- `POST /api/robot/lift/up` - Lift mechanism up
- `POST /api/robot/lift/down` - Lift mechanism down

## Program Structure

Each program consists of:
- `id`: Unique identifier
- `name`: Program name
- `start_node`: Starting node ID
- `actions`: List of actions to execute

### Action Types
1. **Move Action**:
   ```json
   {
     "type": "move",
     "target_node": 5,
     "description": "Move to node 5"
   }
   ```

2. **Lift Action**:
   ```json
   {
     "type": "lift", 
     "lift_action": "up",
     "description": "Lift up mechanism"
   }
   ```

## Robot Requirements

The robot firmware must support:
- HTTP endpoints: `/lift_up`, `/lift_down`, `/turn`, `/drive_dist`
- Navigation between graph nodes
- Lift mechanism control

## Troubleshooting

### Common Issues
1. **Graph not loading**: Make sure you have analyzed topology first
2. **Program execution fails**: Check robot connection and firmware
3. **Lift not working**: Verify lift mechanism is properly connected

### Debug Mode
Enable debug mode in the service:
```python
if __name__ == '__main__':
    app.run(debug=True, port=5002)
```

## Example Workflow

1. **Load Graph**: Analyze topology to create the navigation graph
2. **Set Start Point**: Node 3 (robot's current position)
3. **Set Target Point**: Node 7 (shelf location)
4. **Add Actions**:
   - Move to Node 7
   - Lift Up
   - Move to Node 5
   - Lift Down
   - Move back to Node 3
5. **Save Program**: "Shelf Pickup Task"
6. **Execute**: Robot follows the programmed sequence

This system provides an intuitive way to program complex robot movements without writing code!
