#! /bin/bash

set -e

if [ -z "$1" ]; then
    echo "usage: $0 <rovername>"
    exit -1
fi

TEMPDIR=$(mktemp -d)

function finish {
  rm -rf "$TEMPDIR"
}

trap finish EXIT

# Check that the rover-deploy profile exists. 
if ! catkin profile list | grep -q rover-deploy; then
    echo "Setting up the rover-deploy catkin profile."
    catkin profile add rover-deploy
    catkin profile set rover-deploy
    catkin config --install
    catkin config -x="-rover-deploy"
fi

# Build the current workspace
cd $(catkin locate)
catkin build --profile rover-deploy --no-status --no-color

# Package the build products, scripts, launch configs and stuff
echo "Copying installation files."

# Launch configurations run from the /proj directory by default.
cp -R install-rover-deploy/ $TEMPDIR
cp -R camera_info/ $TEMPDIR
cp -R launch/ $TEMPDIR
cp -R misc/ $TEMPDIR
cat <<EOF > $TEMPDIR/bootstrap.sh
function finish {
  cd ~
  rm -rf "$TEMPDIR"
}

trap finish EXIT

cd $TEMPDIR
pwd
source /opt/ros/kinetic/setup.bash
source ./install-rover-deploy/setup.bash
./misc/rover_onboard_node_launch.sh 

EOF
chmod u+x $TEMPDIR/bootstrap.sh

(
    # Copy the build products to the swarmie. 
    cd $TEMPDIR
    tar -cf - $TEMPDIR 2>/dev/null | ssh robot@$1 "cd /; tar -xf -"
)

echo "Executing rover code."
# Execute the bootstrap code.
ssh -t robot@$1 $TEMPDIR/bootstrap.sh
