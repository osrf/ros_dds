rm -fr c
mkdir c
cd c
idlpp -S -l c ../Chat.idl > /dev/null
echo "Generated C code under '`pwd`'"
cd ..
