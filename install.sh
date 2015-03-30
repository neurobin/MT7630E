BASEDIR=`dirname "${0}"`
cd "$BASEDIR"
sudo chmod -R 777 .
sudo cp -R firmware/Wi-FI/* /lib/firmware/
sudo cp -R firmware/BT/* /lib/firmware/
cd rt2x00
make clean
make
cd ../btloader
make clean
make
##build complete
cd ..
sudo mkdir -p /usr/local/MT7630E
sudo cp -R rt2x00/*.ko /usr/local/MT7630E
sudo cp -R btloader/*.ko /usr/local/MT7630E
sudo cp rt2x00/load.sh /etc/init.d/
sudo update-rc.d load.sh defaults
sudo ./rt2x00/load.sh
