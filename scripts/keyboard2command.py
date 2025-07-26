import keyboard
import json
import time

# 一度だけ反応させたいキーをセットで定義
once_keys = {'w', 'q', 'r', 'p'}

# [key, [command], is_pressed]
commands = [
    ['1', [1,-1,0,0,0,0], False],
    ['2', [1,0,0,0,0,0], False],
    ['3', [1,1,0,0,0,0], False],
    ['4', [0,-1,0,0,0,0], False],
    ['6', [0,1,0,0,0,0], False],
    ['7', [-1,-1,0,0,0,0], False],
    ['8', [-1,0,0,0,0,0], False],
    ['9', [-1,1,0,0,0,0], False],
    ['-', [0,0,1,0,0,0], False],
    ['+', [0,0,-1,0,0,0], False],
    ['0', [0,0,0,1,0,0], False],
    ['.', [0,0,0,-1,0,0], False],
    ['w', [0,0,0,0,-1,0], False],
    ['q', [0,0,0,0,1,0], False],
    ['r', [0,0,0,0,0,1], False],
    ['p', [0,0,0,0,0,-1], False]
]

def on_press(key):
    for i in range(len(commands)):
        if key.name == commands[i][0]:
            if key.name in once_keys:
                # 一度だけ True にする（押しっぱなしは無視）
                if not commands[i][2]:
                    commands[i][2] = True
            else:
                commands[i][2] = True

def on_release(key):
    for i in range(len(commands)):
        if key.name == commands[i][0]:
            if key.name not in once_keys:
                # 通常のキーは押しっぱなしを解除
                commands[i][2] = False

# 登録
for key, _, _ in commands:
    keyboard.on_press_key(key, on_press)
    keyboard.on_release_key(key, on_release)

# メインループ
while True:
    sum_command = [0, 0, 0, 0, 0, 0]
    for _, command, is_pressed in commands:
        if is_pressed:
            sum_command = [x + y for x , y in zip(sum_command, command)]
    with open('key_command.json', 'w') as f:
        json.dump({'command': sum_command}, f)
    print(sum_command)

    # 一度だけ反応させるキーは自動でフラグを False に戻す
    for i in range(len(commands)):
        if commands[i][0] in once_keys:
            commands[i][2] = False

    time.sleep(0.1)