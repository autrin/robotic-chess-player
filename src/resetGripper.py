from multiprocessing import Process
import subprocess
import time

GRIPPER_PROC_ID = None

def is_number(string):
    try:
        float(string)
        return True
    except ValueError:
        return False


def reset_protocol():
    cmd= "ps -h | grep roslaunch | grep robotiq_2f_gripper_control".split(" ")
    print(cmd)
    result = subprocess.run(['ps','-h'], capture_output=True, text=True)
    result = result.stdout.split("\n")
    print()
    for s in result:
        if "robotiq_action_server.launch" in s and 'grep' not in s:
            s = s.split("]")[0].split(" ")
            for p in s:
                if is_number(p):
                    print(p)
                    subprocess.run(['kill', p], capture_output=True, text=True)
                    proc = subprocess.run(['roslaunch', 'robotiq_2f_gripper_control','robotiq_action_server.launch'], capture_output=True, text=True)
                    exit(0)

if __name__ == "__main__":
    p = Process(target=reset_protocol)
    p.start()
    time.sleep(1)
    p.terminate()
    exit(0)


#USAGE
#just copy and paste this line in our actual main.py to reset the gripper automatically
#subprocess.run(['python3','resetGripper.py'], capture_output=True, text=True)

    
   