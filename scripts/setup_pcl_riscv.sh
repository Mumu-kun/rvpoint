#!/usr/bin/env bash
# ==============================================================================
# SCRIPT: scripts/setup_pcl_riscv.sh
# PURPOSE: Unpack official Ubuntu 24.04 pre-compiled RISC-V 64-bit PCL 1.14.0,
#          FLANN, Boost, VTK, OpenNI2, JsonCpp, and all transitive dynamic library dependencies
#          directly into env/deps/pcl.
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEPS_ROOT="${PROJECT_ROOT}/env/deps"
PCL_DIR="${DEPS_ROOT}/pcl"
BUILD_TEMP="/tmp/pcl_deb_cache"

echo "=========================================================================="
echo "   RVPoint: Official Pre-Compiled RISC-V PCL 1.14 Full Dependency Setup   "
echo "=========================================================================="
echo "Destination:     $PCL_DIR"
echo "Package Cache:   $BUILD_TEMP"
echo "=========================================================================="

mkdir -p "$DEPS_ROOT"
mkdir -p "$PCL_DIR"
mkdir -p "$BUILD_TEMP"

PCL_BASE_URL="http://ports.ubuntu.com/ubuntu-ports/pool/universe/p/pcl"
BOOST_BASE_URL="http://ports.ubuntu.com/ubuntu-ports/pool/main/b/boost1.83"
FLANN_BASE_URL="http://ports.ubuntu.com/ubuntu-ports/pool/universe/f/flann"
VTK_BASE_URL="http://ports.ubuntu.com/ubuntu-ports/pool/universe/v/vtk9"

DEB_PACKAGES=(
    # PCL 1.14.0 Official Modules
    "${PCL_BASE_URL}/libpcl-dev_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-common1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-octree1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-kdtree1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-search1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-sample-consensus1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-filters1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-features1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-segmentation1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-io1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-ml1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-surface1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-keypoints1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-registration1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-recognition1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-tracking1.14_1.14.0+dfsg-1_riscv64.deb"
    "${PCL_BASE_URL}/libpcl-stereo1.14_1.14.0+dfsg-1_riscv64.deb"

    # FLANN 1.9.2
    "${FLANN_BASE_URL}/libflann-dev_1.9.2+dfsg-2build1_riscv64.deb"
    "${FLANN_BASE_URL}/libflann1.9_1.9.2+dfsg-2build1_riscv64.deb"

    # Boost 1.83 Headers & Libraries
    "${BOOST_BASE_URL}/libboost1.83-dev_1.83.0-2.1ubuntu3_riscv64.deb"
    "${BOOST_BASE_URL}/libboost-filesystem1.83.0_1.83.0-2.1ubuntu3_riscv64.deb"
    "${BOOST_BASE_URL}/libboost-iostreams1.83.0_1.83.0-2.1ubuntu3_riscv64.deb"
    "${BOOST_BASE_URL}/libboost-system1.83-dev_1.83.0-2.1ubuntu3_riscv64.deb"
    "${BOOST_BASE_URL}/libboost-system1.83.0_1.83.0-2.1ubuntu3_riscv64.deb"
    "${BOOST_BASE_URL}/libboost-program-options1.83.0_1.83.0-2.1ubuntu3_riscv64.deb"
    "${BOOST_BASE_URL}/libboost-thread1.83.0_1.83.0-2.1ubuntu3_riscv64.deb"
    "${BOOST_BASE_URL}/libboost-date-time1.83.0_1.83.0-2.1ubuntu3_riscv64.deb"
    "${BOOST_BASE_URL}/libboost-serialization1.83.0_1.83.0-2.1ubuntu3_riscv64.deb"

    # VTK 9.1 Shared Libraries (Required by libpcl_io.so)
    "${VTK_BASE_URL}/libvtk9.1t64_9.1.0+really9.1.0+dfsg2-7.1build3_riscv64.deb"

    # OpenNI & OpenNI2 Hardware Sensor Plugins (Required by libpcl_io.so)
    "http://ports.ubuntu.com/ubuntu-ports/pool/universe/o/openni2/libopenni2-0_2.2.0.33+dfsg-18_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/universe/o/openni/libopenni0t64_1.5.4.0+dfsg-7.1build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/universe/o/openni-sensor-pointclouds/libopenni-sensor-pointclouds0_5.1.0.41.11-1build2_riscv64.deb"

    # Additional Dynamic Library Dependencies for IO & Math
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libj/libjsoncpp/libjsoncpp25_1.9.5-6build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libu/libusb-1.0/libusb-1.0-0_1.0.27-1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libp/libpcap/libpcap0.8t64_1.10.4-4.1ubuntu3_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libp/libpng1.6/libpng16-16t64_1.6.43-5build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/e/expat/libexpat1_2.6.1-2build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/universe/d/double-conversion/libdouble-conversion3_3.3.0-1build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/x/xz-utils/liblzma5_5.6.1+really5.4.5-1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/f/freetype/libfreetype6_2.13.2+dfsg-1build3_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/g/gcc-14/libgomp1_14.2.0-4ubuntu2~24.04.1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libj/libjpeg-turbo/libjpeg-turbo8_2.1.5-2ubuntu2_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/t/tiff/libtiff6_4.5.1+git230720-4ubuntu2_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libg/libglvnd/libglvnd0_1.7.0-1build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libg/libglvnd/libgl1_1.7.0-1build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libg/libglvnd/libglx0_1.7.0-1build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libg/libglvnd/libopengl0_1.7.0-1build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libg/libglu/libglu1-mesa_9.0.2-1.1build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libx/libxml2/libxml2_2.9.14+dfsg-1.3ubuntu3_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libt/libtheora/libtheora0_1.1.1+dfsg.1-16.1build3_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libo/libogg/libogg0_1.3.5-3build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/universe/h/hdf5/libhdf5-103-1t64_1.10.10+repack-3.1ubuntu4_riscv64.deb"

    # Runtime Compression & System Dependencies
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libg/libgpg-error/libgpg-error0_1.47-3build2_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libw/libwebp/libsharpyuv0_1.3.2-0.4build3_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libg/libgcrypt20/libgcrypt20_1.10.3-2build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libd/libdeflate/libdeflate0_1.19-1build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/j/jbigkit/libjbig0_2.1-6.1ubuntu2_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/l/lerc/liblerc4_4.0.0+ds-4ubuntu2_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libw/libwebp/libwebp7_1.3.2-0.4build3_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/s/systemd/libsystemd0_255.4-1ubuntu8_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libn/libnl3/libnl-3-200_3.7.0-0.3build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libn/libnl3/libnl-route-3-200_3.7.0-0.3build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/universe/o/onetbb/libtbb12_2021.11.0-2ubuntu2_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/s/systemd/libudev1_255.4-1ubuntu8_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libc/libcap2/libcap2_2.66-5ubuntu2_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/r/rdma-core/libibverbs1_50.0-2build2_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/d/dbus/libdbus-1-3_1.14.10-4ubuntu4_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/l/lz4/liblz4-1_1.9.4-1build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/libz/libzstd/libzstd1_1.5.5+dfsg2-2build1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/b/bzip2/libbz2-1.0_1.0.8-5.1build0.1_riscv64.deb"
    "http://ports.ubuntu.com/ubuntu-ports/pool/main/z/zlib/zlib1g_1.3.dfsg-3.1ubuntu2_riscv64.deb"
)

echo "==> Downloading and extracting pre-compiled RISC-V .deb packages..."
for url in "${DEB_PACKAGES[@]}"; do
    deb_file="${BUILD_TEMP}/$(basename "$url")"
    if [ ! -f "$deb_file" ] || [ ! -s "$deb_file" ]; then
        echo "==> Fetching $(basename "$url")..."
        curl -# -L "$url" -o "$deb_file" || wget -q --show-progress "$url" -O "$deb_file"
    else
        echo "==> Using cached $(basename "$url")"
    fi
    echo "    Extracting $(basename "$url")..."
    dpkg -x "$deb_file" "$PCL_DIR"
done

# Ensure standard symlinks and copy libraries to lib/
mkdir -p "${PCL_DIR}/include" "${PCL_DIR}/lib"
if [ -d "${PCL_DIR}/usr/include/pcl-1.14" ]; then
    ln -sf "${PCL_DIR}/usr/include/pcl-1.14" "${PCL_DIR}/include/pcl-1.14"
fi
if [ -d "${PCL_DIR}/usr/include/flann" ]; then
    ln -sf "${PCL_DIR}/usr/include/flann" "${PCL_DIR}/include/flann"
fi
if [ -d "${PCL_DIR}/usr/include/boost" ]; then
    ln -sf "${PCL_DIR}/usr/include/boost" "${PCL_DIR}/include/boost"
fi

# Create unversioned library symlinks recursively and mirror into PCL_DIR/lib
if [ -d "${PCL_DIR}/usr/lib" ]; then
    find "${PCL_DIR}/usr/lib" "${PCL_DIR}/lib" -type f \( -name "*.so*" -o -name "*.a" \) 2>/dev/null | while read -r f; do
        dir=$(dirname "$f")
        base_name=$(basename "$f")
        # Create unversioned .so symlink in the local dir if needed
        if [[ "$base_name" =~ \.so\.[0-9]+ ]]; then
            so_name="${base_name%%.so.*}.so"
            if [ ! -e "${dir}/${so_name}" ]; then
                ln -sf "$base_name" "${dir}/${so_name}"
            fi
            if [ ! -e "${PCL_DIR}/lib/${so_name}" ]; then
                ln -sf "$f" "${PCL_DIR}/lib/${so_name}"
            fi
        fi
        # Symlink versioned .so file into PCL_DIR/lib/
        if [ ! -e "${PCL_DIR}/lib/${base_name}" ]; then
            ln -sf "$f" "${PCL_DIR}/lib/${base_name}"
        fi
    done
fi

echo ""
echo "=========================================================================="
echo "✅ Official Pre-Compiled RISC-V PCL 1.14.0 + Dependencies Installed!"
echo "PCL Headers:   ${PCL_DIR}/usr/include/pcl-1.14"
echo "Boost Headers: ${PCL_DIR}/usr/include/boost"
echo "FLANN Headers: ${PCL_DIR}/usr/include/flann"
echo "Libraries:     ${PCL_DIR}/lib"
echo "=========================================================================="
