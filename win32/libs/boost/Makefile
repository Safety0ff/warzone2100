PKG_NAME:=boost
PKG_VERSION=1_46_0

PKG_SOURCEBASE:=$(PKG_NAME)_$(PKG_VERSION)
PKG_SOURCE:=$(PKG_SOURCEBASE).tar.bz2
PKG_SOURCE_URL:= \
	@SF/boost
PKG_MD5SUM:=37b12f1702319b73876b0097982087e0

# picking an arbitrary one
TARGET:=$(TOPDIR)/build/libs/lib/libboost_thread_win32-mt.a

include $(TOPDIR)/rules.mk

extract-stamp: $(DOWNLOADS)/$(PKG_SOURCE)
	tar xjf $^
	touch extract-stamp

$(PKG_SOURCEBASE)/bootstrap.sh: extract-stamp

# build the bjam program provided with Boost
$(PKG_SOURCEBASE)/bjam: $(PKG_SOURCEBASE)/bootstrap.sh
	cd $(PKG_SOURCEBASE) && ./bootstrap.sh --without-icu 

# FIXME use --version + sed to get version numbers
$(PKG_SOURCEBASE)/user-config.jam:  extract-stamp
	echo "using gcc : 4.4 : i586-mingw32msvc-g++ \
        : \
        <rc>i586-mingw32msvc-windres \
        <archiver>i586-mingw32msvc-ar \
	;" > $(PKG_SOURCEBASE)/user-config.jam

# build boost and install
# Note: a subset of boost can be built using the --with-libraries=list option
$(TARGET): $(PKG_SOURCEBASE)/bjam $(PKG_SOURCEBASE)/user-config.jam
	cd $(PKG_SOURCEBASE) && ./bjam toolset=gcc target-os=windows variant=release threading=multi threadapi=win32 \
	    link=static --prefix=`pwd`/../$(TOPDIR)/build/libs --user-config=user-config.jam $(MAKE_FLAGS) \
	    --without-mpi --without-python -sNO_BZIP2=1 -sNO_ZLIB=1 --layout=tagged install

build: $(TARGET)

clean:
	$(RM) -rf $(PKG_SOURCEBASE) extract-stamp
