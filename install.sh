read -s -p "Enter Password: " mypassword
echo $mypassword | sudo -S cp -R firmware/Wi-FI/* /lib/firmware/
echo $mypassword | sudo -S cp -R firmware/BT/* /lib/firmware/
cd rt2x00
make clean
make
cd ../btloader
make clean
make
##build complete
cd ..
echo $mypassword | sudo -S mkdir -p /usr/local/MT7630E
echo $mypassword | sudo -S cp -R rt2x00/*.ko /usr/local/MT7630E
echo $mypassword | sudo -S cp -R btloader/*.ko /usr/local/MT7630E
echo $mypassword | sudo -S cp rt2x00/load.sh /etc/init.d/
echo $mypassword | sudo -S update-rc.d load.sh defaults
echo $mypassword | sudo -S ./rt2x00/load.sh



