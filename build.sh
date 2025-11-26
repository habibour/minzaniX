#!/bin/bash
# Quick build and run script for minzaniX simulation

set -e

echo "======================================"
echo "  minzaniX 1.0 - Build & Run Script"
echo "======================================"
echo ""

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Clean build directory
if [ "$1" == "clean" ]; then
    echo -e "${BLUE}[*] Cleaning build directory...${NC}"
    rm -rf build
    echo -e "${GREEN}[✓] Clean complete${NC}"
    exit 0
fi

# Create build directory
if [ ! -d "build" ]; then
    echo -e "${BLUE}[*] Creating build directory...${NC}"
    mkdir build
fi

# Build project
echo -e "${BLUE}[*] Configuring CMake...${NC}"
cd build
cmake .. > /dev/null 2>&1

echo -e "${BLUE}[*] Building project...${NC}"
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

if [ $? -eq 0 ]; then
    echo -e "${GREEN}[✓] Build successful!${NC}"
    echo ""
    
    # Run simulation if requested
    if [ "$1" == "run" ]; then
        echo -e "${BLUE}[*] Running simulation...${NC}"
        echo -e "${GREEN}    Press Ctrl+C to stop${NC}"
        echo ""
        ./minzanix_sim
    else
        echo "Build complete. Executable: build/minzanix_sim"
        echo ""
        echo "Usage:"
        echo "  ./build.sh          - Build only"
        echo "  ./build.sh run      - Build and run"
        echo "  ./build.sh clean    - Clean build files"
        echo ""
        echo "To run simulation:"
        echo "  cd build && ./minzanix_sim"
    fi
else
    echo -e "${RED}[✗] Build failed${NC}"
    exit 1
fi
