cd `dirname $0`

rm -rf build

# We're done!
echo Cleaned up the project!

# Compile code.
mkdir -p build
cd build
cmake ..
make -j `nproc` $*

cd ./build
./mpc