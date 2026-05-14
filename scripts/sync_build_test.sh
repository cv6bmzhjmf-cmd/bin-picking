#!/bin/bash
# Bin-Picking 一键同步+编译+测试
# 用法: bash scripts/sync_build_test.sh [launch|test]
set -e
WS=~/ros2_ws
SRC=/mnt/d/pianqian
PKG=bin_picking_sim
PKG_SRC=$WS/src/$PKG

echo "=== 同步代码 ==="
mkdir -p $PKG_SRC/src/vision $PKG_SRC/tests
cp $SRC/src/vision/*.py $PKG_SRC/src/vision/
cp $SRC/src/simulation/launch/sim.launch.py $PKG_SRC/launch/
cp $SRC/tests/*.py $PKG_SRC/tests/ 2>/dev/null || true
echo "同步完成"

echo "=== 编译 ==="
cd $WS
rm -rf build/$PKG install/$PKG
colcon build --packages-select $PKG
source install/setup.bash

MODE=${1:-}
case $MODE in
  launch)
    fuser -k 11345/tcp 2>/dev/null || true
    ros2 launch $PKG sim.launch.py
    ;;
  test)
    echo "--- Geometry ---";  python3 $PKG_SRC/tests/test_geometry.py
    echo "--- Stereo  ---";  python3 $PKG_SRC/tests/test_stereo_matcher.py
    echo "--- Pipeline---";  python3 $PKG_SRC/tests/test_pipeline.py
    ;;
  *)
    echo "编译完成。./sync_build_test.sh launch 启动 | test 离线测试"
    ;;
esac
