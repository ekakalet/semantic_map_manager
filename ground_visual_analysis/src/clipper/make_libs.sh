rm -rfv build/*
rmdir build
mkdir build
cd build
cmake ..
make
sudo make install
