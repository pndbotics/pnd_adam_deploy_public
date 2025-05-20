appname=`basename $0 | sed s,\.sh$,,`

dirname=`dirname $0`
tmp="${dirname#?}"

if [ "${dirname%$tmp}" != "/" ]; then
dirname=$PWD/$dirname
fi
LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH

LD_LIBRARY_PATH=/opt/ros/humble/lib/:$LD_LIBRARY_PATH
LD_LIBRARY_PATH=/opt/ros/humble/opt/yaml_cpp_vendor/lib/:$LD_LIBRARY_PATH
LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib/:$LD_LIBRARY_PATH
LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu/:$LD_LIBRARY_PATH
LD_LIBRARY_PATH=../robotPublisher/install/robotstatepub/lib/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH
$dirname/$appname "$@"
