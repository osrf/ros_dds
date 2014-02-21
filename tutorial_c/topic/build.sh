rm -fr c
mkdir c
cd c
idlpp -S -l c ../Chat.idl
echo "Generated C code under '`pwd`'"
cd ..

rm -fr build
mkdir build
cd build
cmake ..
make
cd ..
