build_dir="build"
install_dir="install"
sim_dir="sim"

cd ..
source_dir=$(pwd);

mkdir ${build_dir}
mkdir ${install_dir}

pushd ${build_dir} > /dev/null
cmake -DCMAKE_INSTALL_PREFIX=$source_dir/$install_dir \
      -DCMAKE_BUILD_TYPE=Release \
      -DUSE_TESTS=TRUE \
      $source_dir
make
make install

git clone https://github.com/eeros-project/sim-eeros.git $sim_dir
pushd ${sim_dir} > /dev/null
git checkout master
mkdir ${build_dir}
pushd ${build_dir} > /dev/null
cmake -DCMAKE_INSTALL_PREFIX=$source_dir/$install_dir \
      -DCMAKE_BUILD_TYPE=Release \
      ..
make
make install
popd > /dev/null
popd > /dev/null

pushd test > /dev/null
export LD_LIBRARY_PATH=LD_LIBRARY_PATH:$source_dir/$install_dir/lib
echo ${LD_LIBRARY_PATH}
./unitTests -l sim
popd > /dev/null

popd > /dev/null

