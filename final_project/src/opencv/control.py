import keyboard

while True:
    print("123")
    print(keyboard.read_key(), "123")
    if keyboard.read_key() == "p":
        print("You pressed p")
        break