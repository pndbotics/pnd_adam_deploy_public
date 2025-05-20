##init path
root=$PWD
sudo cp ./libfolder/lib* /usr/lib/x86_64-linux-gnu/
sudo chmod 777 /usr/lib/x86_64-linux-gnu/libssl.so.1.1
sudo chmod 777 /usr/lib/x86_64-linux-gnu/libcrypto.so.1.1
##build mujoco
check_mujoco="/usr/local/lib/libmujoco.so"
if [ -f "$check_mujoco" ]; then
    echo "File $check_mujoco already exists, skipping download and build."
else
    echo "Do you want to build MuJoCo?"
    echo "y) yes"
    echo "n) no"
    read -p "Enter y or n: " option

    case $option in
    y)
        url_mujoco="https://github.com/google-deepmind/mujoco/archive/refs/tags/3.2.0.zip"
        wget $url_mujoco
        if [ $? -ne 0 ]; then
            echo "wget error, please check your network then retry."
            exit 1
        fi

        unzip 3.2.0.zip
        if [ $? -ne 0 ]; then
            echo "Failed to unzip 3.2.0.zip."
            exit 1
        fi

        cd mujoco-3.2.0 || {
            echo "Failed to change directory to mujoco-main"
            exit 1
        }
        mkdir build && cd build || {
            echo "Failed to change directory to build"
            exit 1
        }

        cmake ..
        if [ $? -ne 0 ]; then
            echo "cmake configuration failed."
            exit 1
        fi

        make -j8
        if [ $? -ne 0 ]; then
            echo "make failed."
            exit 1
        fi

        make install
        cd "$root" || exit 1
        rm 3.2.0.zip
        rm -rf mujoco-3.2.0
        ;;
    n)
        echo "Skipping MuJoCo build."
        ;;
    *)
        echo "Please enter y or n."
        ;;
    esac
fi

##build rbdl
check_file="/usr/local/lib/librbdl.so.3.2.0"

if [ -f "$check_file" ]; then
    echo "File $check_file already exists, skipping download and build."
else
    url_rbdl="https://github.com/rbdl/rbdl/archive/refs/tags/v3.2.0.zip"
    wget $url_rbdl
    if [ $? -ne 0 ]; then
        echo "wget error, please check your network then retry"
        exit 1
    fi

    unzip v3.2.0.zip
    if [ $? -ne 0 ]; then
        echo "Failed to unzip master.zip"
        exit 1
    fi
    cd rbdl-3.2.0 || exit 1
    mkdir build && cd build
    cmake ..
    if [ $? -ne 0 ]; then
        echo "cmake configuration failed"
        exit 1
    fi

    make -j8
    if [ $? -ne 0 ]; then
        echo "make failed"
        exit 1
    fi

    make install

    cd $root
    rm -rf rbdl-3.2.0
    rm v3.2.0.zip
fi

##install libtorch
check_torch="$root/libtorch"
if [ -d "$check_torch" ]; then
    echo "Dir $check_torch already exists"
else
    url_libtorch=https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.12.1%2Bcpu.zip
    wget $url_libtorch
    if [ $? -ne 0 ]; then
        echo "wget erro please check your network then retry"
    fi
    unzip libtorch-cxx11-abi-shared-with-deps-1.12.1+cpu.zip
    rm -rf libtorch-cxx11-abi-shared-with-deps-1.12.1+cpu.zip
fi

##build qpOASES
check_qpOASES="/usr/local/lib/libqpOASES.a"
if [ -f "$check_qpOASES" ]; then
    echo "File $check_qpOASES already exists, skipping download and build."
else
    url_qpOASES=https://github.com/coin-or/qpOASES/archive/refs/heads/master.zip
    wget $url_qpOASES
    if [ $? -ne 0 ]; then
        echo "wget erro please check your network then retry"
    fi

    unzip master.zip
    cd qpOASES-master
    mkdir build && cd build

    cmake ..
    if [ $? -ne 0 ]; then
        echo "cmake configuration failed"
        exit 1
    fi

    make -j8
    if [ $? -ne 0 ]; then
        echo "make failed"
        exit 1
    fi

    make install
    cd $root
    rm -rf qpOASES-master
    rm master.zip
fi

##install eigen
check_eigen="/usr/local/include/eigen3"
if [ -d "$check_eigen" ]; then
    echo "Dir $check_eigen already exists"
else
    url_eigen=https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    wget $url_eigen
    if [ $? -ne 0 ]; then
        echo "wget erro please check your network then retry"
    fi
    tar -xzvf eigen-3.4.0.tar.gz
    cd eigen-3.4.0
    mkdir build && cd build
    cmake ..
    if [ $? -ne 0 ]; then
        echo "cmake configuration failed"
        exit 1
    fi
    make install
    cd $root
    rm eigen-3.4.0.tar.gz
    rm -rf eigen-3.4.0
fi
