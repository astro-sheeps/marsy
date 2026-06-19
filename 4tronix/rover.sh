echo Creating marsrover folder
if [ ! -d ~/marsrover ]; then
  mkdir ~/marsrover
fi
cd ~/marsrover
echo Copying MARS Rover Library Module
wget -q http://4tronix.co.uk/rover/rover.py -O rover.py
echo Copying Test Files
wget -q http://4tronix.co.uk/rover/ledTest.py -O ledTest.py
wget -q http://4tronix.co.uk/rover/motorTest.py -O motorTest.py
wget -q http://4tronix.co.uk/rover/pca9685.py -O pca9685.py
wget -q http://4tronix.co.uk/rover/servoTest.py -O servoTest.py
wget -q http://4tronix.co.uk/rover/sonarTest.py -O sonarTest.py
wget -q http://4tronix.co.uk/rover/keypad.py -O keypad.py
wget -q http://4tronix.co.uk/rover/driveRover.py -O driveRover.py
wget -q http://4tronix.co.uk/rover/calibrateServos.py -O calibrateServos.py

echo M.A.R.S. Rover Files Copied


