include $(TOPDIR)/rules.mk

PKG_NAME:=intel-microcode
PKG_VERSION:=20191115
PKG_RELEASE:=2

PKG_SOURCE:=intel-microcode_3.$(PKG_VERSION).$(PKG_RELEASE).tar.xz
PKG_SOURCE_URL:=https://dl.meraki.net
PKG_MD5SUM:=0ec32d042500d4b5b8c7c460394ffde9
PKG_CAT:=xzcat

PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)-3.$(PKG_VERSION).$(PKG_RELEASE)
PKG_INSTALL_DIR:=$(PKG_BUILD_DIR)/ipkg-install

ifdef CONFIG_TARGET_x86_64
        MICROCODE:="intel-microcode-64"
else
        MICROCODE:="intel-microcode"
endif

include $(TOPDIR)/package/rules.mk

$(eval $(call PKG_template,INTELMICROCODE,intel-microcode,$(PKG_VERSION)-$(PKG_RELEASE),$(ARCH)))

$(PKG_BUILD_DIR)/.configured:
	touch $@

$(PKG_BUILD_DIR)/.built:
	IUCODE_TOOL=$(STAGING_DIR)/usr/sbin/iucode_tool \
	$(MAKE) -C $(PKG_BUILD_DIR) \
		CC="$(TARGET_CC)" \
		DESTDIR="$(PKG_INSTALL_DIR)"
	# At this point, we need to manually move the correct microcode blob from
	# $(PKG_BUILD_DIR)/intel-ucode/" into "$(TOPDIR)/../linux-*.*/firmware/intel-ucode"
	# so that the microcode will be built into the kernel. We only need
	# to do this step once when we need to apply a new microcode revision.
	touch $@
