loongson-sources
====================
loongson-sources is a patchset against offical upstream Linux kernel for YeeLoong 8089D with Loongson 2F processor.


Features
----------
* genpatches
* BFS
* BFQ
* CJKtty
* UKSM
* exfat
* colored kernel message output
* 2D accelerated sm712fb framebuffer
* YeeLoong Power Management Driver
* YeeLoong Fn keys Driver
* Out-of-tree bugfixes

Binary
----------

We use [GitHub Release](https://github.com/biergaizi/loongson-sources/releases) to publish the latest binary kernel and GPG siguature. The naming format is:

	linux-[kernel version]-yeeloong-gaizi.tar.xz
	linux-[kernel version]-yeeloong-gaizi.tar.xz.sig

Build from Source
--------------------

* First, I assume you have the knowledge on cross-compiling and related toolchains.

* Download the mainline kernel of the same series as loongson-sources, without any patch level.

  For example, if you want to build loongson-sources-3.15.3, you should download
  Linux 3.15.0 from kernel.org.

* Copy `patches` directory to the root directory of the kernel.

* Run the `./patches/patch-it` shell script to patch the kernel.

* copy `./patches/config` to the root directory, rename it to `.config`

* make (you should setup the cross-toolchains correctly)

* You're down. You can use the `./patches/packaging.sh` to generate a tarball if you want.
