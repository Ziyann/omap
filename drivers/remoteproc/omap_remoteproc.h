/*
 * Remote processor messaging
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name Texas Instruments nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _OMAP_RPMSG_H
#define _OMAP_RPMSG_H

/*
 * enum - Predefined Mailbox Messages
 *
 * @RP_MBOX_READY: informs the M3's that we're up and running. this is
 * part of the init sequence sent that the M3 expects to see immediately
 * after it is booted.
 *
 * @RP_MBOX_PENDING_MSG: informs the receiver that there is an inbound
 * message waiting in its own receive-side vring. please note that currently
 * this message is optional: alternatively, one can explicitly send the index
 * of the triggered virtqueue itself. the preferred approach will be decided
 * as we progress and experiment with those two different approaches.
 *
 * @RP_MBOX_CRASH: this message is sent if BIOS crashes
 *
 * @RP_MBOX_ECHO_REQUEST: a mailbox-level "ping" message.
 *
 * @RP_MBOX_ECHO_REPLY: a mailbox-level reply to a "ping"
 *
 * @RP_MBOX_ABORT_REQUEST: a "please crash" request, used for testing the
 * recovery mechanism (to some extent).
 *
 * @RP_MBOX_BOOTINIT_DONE: this message is sent by remote processor once it has
 * completed some essential initialization during its boot. This notification
 * can be used to do state maintainance.
 *
 * @RP_MBOX_SUSPEND: suspend request for the remote processor
 *
 * @RP_MBOX_SUSPEND_FORCED: forced suspend for system suspend request
 *
 * @RP_MBOX_SUSPEND_ACK: remote processor acked suspend request
 *
 * @P_MBOX_SUSPEND_CANCEL: remote processor canceled suspend request
 *
 * @RP_MBOX_READY_FOR_NS_CREATE: notify remote processor that
 * the rpmsg_ns_cb endpoint is registered and we are ready to
 * receive message with flag NS_CREATE for creating rpmsg-omx1 channel.
 *
 * Intoduce new message definitions if any here.
 *
 * @RP_MBOX_END_MSG: Indicates end of known/defined messages from remote core
 * This should be the last definition.
 *
 */
enum omap_rp_mbox_messages {
	RP_MBOX_READY		= 0xFFFFFF00,
	RP_MBOX_PENDING_MSG	= 0xFFFFFF01,
	RP_MBOX_CRASH		= 0xFFFFFF02,
	RP_MBOX_ECHO_REQUEST	= 0xFFFFFF03,
	RP_MBOX_ECHO_REPLY	= 0xFFFFFF04,
	RP_MBOX_ABORT_REQUEST	= 0xFFFFFF05,
	RP_MBOX_BOOTINIT_DONE	= 0xFFFFFF07,
	RP_MBOX_SUSPEND		= 0xFFFFFF10,
	RP_MBOX_SUSPEND_FORCED	= 0xFFFFFF11,
	RP_MBOX_SUSPEND_ACK	= 0xFFFFFF12,
	RP_MBOX_SUSPEND_CANCEL	= 0xFFFFFF13,
#ifdef CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_SOHO
	RP_MBOX_READY_FOR_NS_CREATE = 0xFFFFFF14,
	RP_MBOX_END_MSG		= 0xFFFFFF15,
#else
	RP_MBOX_END_MSG		= 0xFFFFFF14,
#endif
};

/**
 * enum - OMAP specific resources
 *	  resources specific to omap can be appended here
 *
 * @OMAP_RSC_HWSPIN: Resource holding information on hwspinlock state
 *
 * Introduce new custom resource definitions here
 *
 * @OMAP_RSC_MAX: Indicates end of known/defined omap resources
 * This should be the last definition.
 *
 */
enum omap_resources {
	OMAP_RSC_HWSPIN	= 1,
	OMAP_RSC_MAX	= 2,
};

/**
 * struct fw_rsc_custom_spinlock - custom resource to define spinlock
 * @num_locks_da: device address of hwspinlocks number
 * @da: device address of hwspinlock state array
 * @name: name of the resource
 *
 */
struct fw_rsc_custom_spinlock {
	u32 num_locks_da;
	u32 da;
	char name[32];
};

#endif /* _OMAP_RPMSG_H */
