import requests
import time

delay = 0.1 # sec

# main functino for testing the comms module
def main():
    print("Testing comms module")
    robot_url = "http://parrot-robot1.wifi.local.cmu.edu"

    for i in range(1,11):
        json = {"dtype": "pallet", 
                "power": i % 2
                }
        x = requests.post(robot_url, data=json)
        time.sleep(delay)
    
    return 0    
    

# call main function on start
if __name__ == '__main__':
    main()