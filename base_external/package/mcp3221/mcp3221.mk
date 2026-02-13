################################################################################
#
# mcp3221
#
################################################################################

MCP3221_VERSION = 1.0
MCP3221_SITE = $(BR2_EXTERNAL_assignment_base_PATH)/package/mcp3221/src
MCP3221_SITE_METHOD = local

define MCP3221_BUILD_CMDS
	$(TARGET_CC) $(TARGET_CFLAGS) -O2 -Wall \
		-o $(@D)/mcp3221_read $(@D)/mcp3221_read.c
endef

define MCP3221_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/mcp3221_read $(TARGET_DIR)/usr/bin/mcp3221_read
endef

$(eval $(generic-package))
