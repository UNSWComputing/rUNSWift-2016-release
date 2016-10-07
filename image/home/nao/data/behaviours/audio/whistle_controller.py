import os


def kill_all_python_processes():
   # killall python processes https://stackoverflow.com/questions/18428
   # Updated to:
   # - Only kill whistle_detector processes
   # - Use a bash for loop to get around kill with no args printing stuff
   #os.system("kill $(ps aux | grep '[w]histle_detector' | awk '{print $2}')")
   os.system("kill $(sudo -S ps aux | grep [w]histle_detector | awk '{print $2}')")

def start_listening_for_whistles():
   # Start new background Python process to listen for whistles
   # and save detected whistle files in NAO_WHISTLE_LOCATION
   os.system('/usr/bin/amixer -qs < /home/nao/data/volumeinfo.dat')
   os.system('python $HOME/whistle/whistle_detector.py &')
