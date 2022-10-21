import sys
import time
import keyboard

def hi():
    print("hi")
    time.sleep(1)
    if keyboard.add_hotkey('q',lambda: sys.exit()):
        print("pressed")
        sys.exit()
while True:
    hi()