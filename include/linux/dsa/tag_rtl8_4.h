/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _NET_DSA_TAG_RTL8_4_H
#define _NET_DSA_TAG_RTL8_4_H

#include <linux/types.h>

/**
 * struct rtl8_4_tagger_data - switch-specific data of the rtl8_4 tagger
 * @igmp_mld_trap_reason: REASON value with which the switch delivers IGMP
 *	and MLD messages it trapped to the CPU, or 0 if it does not trap
 *	them. Such frames were not forwarded by the switch.
 */
struct rtl8_4_tagger_data {
	u8 igmp_mld_trap_reason;
};

#endif /* _NET_DSA_TAG_RTL8_4_H */
