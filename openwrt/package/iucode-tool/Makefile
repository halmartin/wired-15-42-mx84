include $(TOPDIR)/rules.mk

PKG_NAME:=iucode-tool
PKG_VERSION:=2.3.1
PKG_RELEASE:=1
PKG_MD5SUM:=63b33cc0ea1f8c73b443412abbf39d6f

PKG_SOURCE:=iucode-tool_$(PKG_VERSION).tar.xz
PKG_SOURCE_URL:=https://dl.meraki.net
PKG_CAT:=xzcat

PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)
PKG_INSTALL_DIR:=$(PKG_BUILD_DIR)/ipkg-install

PKG_MAINTAINER:=Zoltan HERPAI <wigyori@uid0.hu>
PKG_LICENSE:=GPL-2.0

include $(TOPDIR)/package/rules.mk

$(eval $(call PKG_template,IUCODETOOL,iucode-tool,$(PKG_VERSION)-$(PKG_RELEASE),$(ARCH)))

$(PKG_BUILD_DIR)/.configured:
	(cd $(PKG_BUILD_DIR); rm -rf config.{cache,status} ;\
		./configure \
		--prefix=/usr \
	);
	touch $@

$(PKG_BUILD_DIR)/.built:
	$(MAKE) -C $(PKG_BUILD_DIR) \
		DESTDIR="$(PKG_INSTALL_DIR)" \
		install
	touch $@

install-dev: $(PKG_BUILD_DIR)/.built
	install -m0755 -d $(STAGING_DIR)/usr/sbin
	$(CP) $(PKG_BUILD_DIR)/iucode_tool $(STAGING_DIR)/usr/sbin/

uninstall-dev:
	 rm -f \
	 $(STAGING_DIR)/usr/sbin/iucode-tool \

compile-targets: install-dev
clean-targets: uninstall-dev
