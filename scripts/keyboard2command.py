import keyboard
import json
import time

commands = [
    ['1', [1,-1,0,0]],
    ['2', [1,0,0,0]],
    ['3', [1,1,0,0]],
    ['4', [0,-1,0,0]],
    ['6', [0,1,0,0]],
    ['7', [-1,-1,0,0]],
    ['8', [-1,0,0,0]],
    ['9', [-1,1,0,0]],
    ['-', [0,0,1,0]],
    ['+', [0,0,-1,0]],
    ['divide', [0,0,0,1]],
    ['multiply', [0,0,0,-1]]
]

while True:
    sum_command = [0, 0, 0, 0]
    for key, command in commands:
        if keyboard.is_pressed(key):
            sum_command = [x + y for x , y in zip(sum_command, command)]
    with open('key_command.json', 'w') as f:
        json.dump({'command': sum_command}, f)
    print(sum_command)
    time.sleep(0.1)