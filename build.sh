vcpkg install
if [ -d "build" ]; then
    rm -rf build
fi
mkdir build
cd build
cmake ..
cmake --build .
ctest . --rerun-failed --output-on-failure