import requests
import time

# main functino for testing the comms module
def main():
    print("Testing comms module")
    robot_url = "http://172.26.170.242"
    
    # get user input for rgb values
    r = input("Enter red value: ")
    g = input("Enter green value: ")
    b = input("Enter blue value: ")
    
    # loop through index 0 to 11
    for i in range(12):
        json = {"dtype": "rgbi", 
                "r": r,
                "g":g,
                "b": b,
                "index": str(i)
                }
        x = requests.post(robot_url, data=json)
        time.sleep(0.5)
    
    # turn off the lights
    for i in range(12):
        json = {"dtype": "rgbi",
                "r": "0",
                "g":"0",
                "b": "0",
                "index": str(i)
                }
        x = requests.post(robot_url, data=json)
        time.sleep(0.5)
    
    return 0    
    

# call main function on start
if __name__ == '__main__':
    while True:
        main()