#ifndef SMB136_CHARGER_H
#define SMB136_CHARGER_H

#include <linux/types.h>
#include <linux/power_supply.h>

struct smb136_charger_platform_data {
	char		**supplied_to;
	size_t		num_supplicants;
};

#endif /* SMB136_CHARGER_H */
