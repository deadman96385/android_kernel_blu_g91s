export PATH=$PWD/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin:$PWD/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9.1/bin:$PWD/prebuilts/gcc/linux-x86/aarch64/gcc-arm-8.3-2019.03-x86_64-aarch64-elf/bin:$PWD/prebuilts/gcc/linux-x86/host/x86_64-w64-mingw32-4.8/bin:$PWD/prebuilts/gcc/linux-x86/host/x86_64-linux-glibc2.17-4.8/bin:$PATH

        cd kernel-4.14
        make ARCH=arm64 O=out k69v1_64_defconfig
        make ARCH=arm64 -Wno-error  O=out  -j32 2>&1 | tee build_kernel.log

