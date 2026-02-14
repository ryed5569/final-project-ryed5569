################################################################################
#
# mcp3221
#
################################################################################

MCP3221_VERSION = 1.0
MCP3221_SITE = $(BR2_EXTERNAL_assignment_base_PATH)/package/mcp3221/src
MCP3221_SITE_METHOD = local

MCP3221_DEPENDENCIES = sqlite

define MCP3221_BUILD_CMDS
	$(TARGET_CC) $(TARGET_CFLAGS) -O2 -Wall -pthread \
		-o $(@D)/mcp3221_read $(@D)/mcp3221_read.c

	$(TARGET_CC) $(TARGET_CFLAGS) -O2 -Wall -pthread \
		-I$(@D) \
		-o $(@D)/mcp3221d \
		$(@D)/mcp3221d.c \
		$(TARGET_LDFLAGS) -lsqlite3
endef

define MCP3221_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/mcp3221_read $(TARGET_DIR)/usr/bin/mcp3221_read
	$(INSTALL) -D -m 0755 $(@D)/mcp3221d     $(TARGET_DIR)/usr/bin/mcp3221d
endef

$(eval $(generic-package))
