rm ./*.ko ./*.o
make -C /lib/modules/$(uname -r)/build/ M=$PWD
