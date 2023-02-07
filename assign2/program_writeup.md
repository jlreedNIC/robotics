# Random Walk program layout

## Block Diagram

![Block Diagram]("Algorithm_flowchart_example.png")

## Algorithms

### random_walk()
```python
if counter less than 20:
    start_walking()
else:
    go_home()
    dock()
```

### start_walking()
```python
while counter less than 20:
    rotate random direction
        spin(node)
    move forward .75 meters
        spin(node)
    counter++
```

### feedback_callback()
```python
update (x,y) position
update isHome
if crash_detected:
    crash_detected_callback()
```

### isRobotHome()
```python
if x < 1 and y < 1:
    isHome = True
else:
    isHome = False
```

### crash_detected_callback()
```python
cancel_current_goal()

rotate 180 degrees
    spin(node)
move forward .5m
    spin(node)

'''instead of initiating random walk again, would it be easier to turn 90 degrees and try again?'''
if counter >= 20:
    counter = 19 # ensures robot will move out of way of obstacle when trying to go home

random_walk()
```

### go_home()
```python
if not isHome:
    '''might need to handle going around front of dock
       something like if x<home.x: move in front
       if y<home.y: move in front'''
    angle = calculate direction based on position

    distance = calculate meters to travel based on position # remember to subtract off a little to hit the 'sweet spot' for docking

    rotate(angle)
        spin(node)
    move_forward(distance)
        spin(node)
```

### dock()
```python
start docking
```