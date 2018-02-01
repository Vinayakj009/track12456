INIT_FILE="GOLD_RUSH.sh"
SERVICE_FILE="gold_rush"
SERVICE_FOLDER="Gold_Rush"
SERVICE_NAME="GOLD_RUSH"
OTHER_REQ=""
DEPENDENCIES=""
START_DIRECTORY=$(pwd)
#echo "Installing required dependencies"
#sudo apt-get install $DEPENDENCIES
cp ../dist/Debug/GNU-Linux/track12456 gold_rush
echo "Deleting old services folder"
sudo rm -r /usr/src/$SERVICE_FOLDER
echo "Creating new services folder"
sudo mkdir -p /usr/src/$SERVICE_FOLDER
echo "Copying essentials to services folder"
sudo cp *.txt /usr/src/$SERVICE_FOLDER/
sudo cp $SERVICE_FILE /usr/src/$SERVICE_FOLDER/
sudo cp $INIT_FILE /usr/src/$SERVICE_FOLDER/
echo "Entering services folder"
cd /usr/src/$SERVICE_FOLDER
echo "Changing permission of the service file"
sudo chmod +x $SERVICE_FILE
echo "Copying init file to /etc/init.d"
sudo cp $INIT_FILE /etc/init.d/
echo "Setting up service"
sudo chmod 777 /etc/init.d/$INIT_FILE
sudo update-rc.d $INIT_FILE defaults
echo "Returning to installation folder"
cd $START_DIRECTORY
$OTHER_REQ
echo "Enabling service to run on boot"
sudo service $INIT_FILE restart
sudo update-rc.d $INIT_FILE enable
