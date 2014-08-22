/**
 * Copyright (c) 2011 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __TF_TEE_PROTOCOL_H__
#define __TF_TEE_PROTOCOL_H__

/*----------------------------------------------------------------------------
 *
 * This header file defines the structure used in the SChannel Protocol.
 * See your Product Reference Manual for a specification of the SChannel
 * protocol.
 *---------------------------------------------------------------------------*/

/*
 * representation of an UUID.
 */
struct tf_tee_uuid {
	u32 time_low;
	u16 time_mid;
	u16 time_hi_and_version;
	u8 clock_seq_and_node[8];
};


/**
 * Command parameters.
 */
struct tf_tee_command_param_value {
	u32    a;
	u32    b;
};

struct tf_tee_command_param_temp_memref {
	u32    descriptor; /* data pointer for exchange message.*/
	u32    size;
	u32    offset;
};

struct tf_tee_command_param_memref {
	u32      block;
	u32      size;
	u32      offset;
};

union tf_tee_command_param {
	struct tf_tee_command_param_value             value;
	struct tf_tee_command_param_temp_memref       temp_memref;
	struct tf_tee_command_param_memref            memref;
};

/**
 * Answer parameters.
 */
struct tf_tee_answer_param_value {
	u32   a;
	u32   b;
};

struct tf_tee_answer_param_size {
	u32   _ignored;
	u32   size;
};

union tf_tee_answer_param {
	struct tf_tee_answer_param_size    size;
	struct tf_tee_answer_param_value   value;
};

/* TF_MAX_COARSE_PAGES is the number of level 1 descriptors (describing
 * 1MB each) that can be shared with the secure world in a single registered
 * shared memory block. It must be kept in synch with
 * SCHANNEL6_MAX_DESCRIPTORS_PER_REGISTERED_SHARED_MEM in the SChannel
 * protocol spec. */
#define TF_TEE_MAX_COARSE_PAGES_8DESC                 8
#define TF_TEE_MAX_COARSE_PAGES                     128

/*
 * Shared mem flags
 */
#define TF_TEE_SHARED_MEM_FLAG_INPUT   1
#define TF_TEE_SHARED_MEM_FLAG_OUTPUT  2
#define TF_TEE_SHARED_MEM_FLAG_INOUT   3


/*
 * Parameter types
 */
#define TF_TEE_PARAM_TYPE_NONE               0x0
#define TF_TEE_PARAM_TYPE_VALUE_INPUT        0x1
#define TF_TEE_PARAM_TYPE_VALUE_OUTPUT       0x2
#define TF_TEE_PARAM_TYPE_VALUE_INOUT        0x3
#define TF_TEE_PARAM_TYPE_MEMREF_TEMP_INPUT  0x5
#define TF_TEE_PARAM_TYPE_MEMREF_TEMP_OUTPUT 0x6
#define TF_TEE_PARAM_TYPE_MEMREF_TEMP_INOUT  0x7
#define TF_TEE_PARAM_TYPE_MEMREF_ION_HANDLE  0xB
#define TF_TEE_PARAM_TYPE_MEMREF_INPUT       0xD
#define TF_TEE_PARAM_TYPE_MEMREF_OUTPUT      0xE
#define TF_TEE_PARAM_TYPE_MEMREF_INOUT       0xF

#define TF_TEE_PARAM_TYPE_MEMREF_FLAG               0x4
#define TF_TEE_PARAM_TYPE_REGISTERED_MEMREF_FLAG    0x8

#define TF_TEE_MAKE_PARAM_TYPES(t0, t1, t2, t3) \
	((t0) | ((t1) << 4) | ((t2) << 8) | ((t3) << 12))
#define TF_TEE_GET_PARAM_TYPE(t, i) (((t) >> (4 * i)) & 0xF)


/*
 *  return origins
 */
#define TF_TEE_ORIGIN_COMMS       2
#define TF_TEE_ORIGIN_TEE         3
#define TF_TEE_ORIGIN_TRUSTED_APP 4
/*
 * The message types.
 */
#define TF_TEE_MESSAGE_TYPE_CREATE_DEVICE_CONTEXT   0x02
#define TF_TEE_MESSAGE_TYPE_DESTROY_DEVICE_CONTEXT  0xFD
#define TF_TEE_MESSAGE_TYPE_REGISTER_SHARED_MEMORY  0xF7
#define TF_TEE_MESSAGE_TYPE_RELEASE_SHARED_MEMORY   0xF9
#define TF_TEE_MESSAGE_TYPE_OPEN_CLIENT_SESSION     0xF0
#define TF_TEE_MESSAGE_TYPE_CLOSE_CLIENT_SESSION    0xF2
#define TF_TEE_MESSAGE_TYPE_INVOKE_CLIENT_COMMAND   0xF5
#define TF_TEE_MESSAGE_TYPE_CANCEL_CLIENT_COMMAND   0xF4
#define TF_TEE_MESSAGE_TYPE_MANAGEMENT              0xFE

struct tf_tee_command_header {
	u8                       message_size;
	u8                       message_type;
	u16                      message_info;
	u32                      operation_id;
};

struct tf_tee_answer_header {
	u8                   message_size;
	u8                   message_type;
	u16                  message_info;
	u32                  operation_id;
	u32                  error_code;
};

/*
 * CREATE_DEVICE_CONTEXT command message.
 */
struct tf_tee_command_create_device_context {
	u8                       message_size;
	u8                       message_type;
	u16                      message_info_rfu;
	u32                      operation_id;
	u32                      device_context_id;
};

/*
 * CREATE_DEVICE_CONTEXT answer message.
 */
struct tf_tee_answer_create_device_context {
	u8                       message_size;
	u8                       message_type;
	u16                      message_info_rfu;
	/* an opaque Normal World identifier for the operation */
	u32                      operation_id;
	u32                      error_code;
	/* an opaque Normal World identifier for the device context */
	u32                      device_context;
};

/*
 * DESTROY_DEVICE_CONTEXT command message.
 */
struct tf_tee_command_destroy_device_context {
	u8                       message_size;
	u8                       message_type;
	u16                      message_info_rfu;
	u32                      operation_id;
	u32                      device_context;
};

/*
 * DESTROY_DEVICE_CONTEXT answer message.
 */
struct tf_tee_answer_destroy_device_context {
	u8                       message_size;
	u8                       message_type;
	u16                      message_info_rfu;
	/* an opaque Normal World identifier for the operation */
	u32                      operation_id;
	u32                      error_code;
	u32                      device_context_id;
};

/*
 * OPEN_CLIENT_SESSION command message.
 */
struct tf_tee_command_open_client_session {
	u8                            message_size;
	u8                            message_type;
	u16                           param_types;
	/* an opaque Normal World identifier for the operation */
	u32                           operation_id;
	u32                           device_context;
	u32                           cancellation_id;
	u64                           timeout;
	struct tf_tee_uuid                destination_uuid;
	union tf_tee_command_param        params[4];
	u32                           login_type;
	/*
	 * Size = 0 for public, [16] for group identification, [20] for
	 * authentication
	 */
	u8                            login_data[20];
};

/*
 * OPEN_CLIENT_SESSION answer message.
 */
struct tf_tee_answer_open_client_session {
	u8                       message_size;
	u8                       message_type;
	u8                       error_origin;
	u8                       __reserved;
	/* an opaque Normal World identifier for the operation */
	u32                      operation_id;
	u32                      error_code;
	u32                      client_session;
	union tf_tee_answer_param    answers[4];
};

/*
 * CLOSE_CLIENT_SESSION command message.
 */
struct tf_tee_command_close_client_session {
	u8                       message_size;
	u8                       message_type;
	u16                      message_info_rfu;
	/* an opaque Normal World identifier for the operation */
	u32                      operation_id;
	u32                      device_context;
	u32                      client_session;
};

/*
 * CLOSE_CLIENT_SESSION answer message.
 */
struct tf_tee_answer_close_client_session {
	u8                       message_size;
	u8                       message_type;
	u16                      message_info_rfu;
	/* an opaque Normal World identifier for the operation */
	u32                      operation_id;
	u32                      error_code;
};


/*
 * REGISTER_SHARED_MEMORY command message
 */
struct tf_tee_command_register_shared_memory_128 {
	u8  message_size;
	u8  message_type;
	u16 memory_flags;
	u32 operation_id;
	u32 device_context;
	u32 block_id;
	u32 shared_mem_size;
	u32 shared_mem_start_offset;
	u32 shared_mem_descriptors[TF_MAX_COARSE_PAGES];
};

struct tf_tee_command_register_shared_memory_8 {
	u8  message_size;
	u8  message_type;
	u16 memory_flags;
	u32 operation_id;
	u32 device_context;
	u32 block_id;
	u32 shared_mem_size;
	u32 shared_mem_start_offset;
	u32 shared_mem_descriptors[TF_MAX_COARSE_PAGES_8DESC];
};

struct tf_tee_command_register_shared_memory_1 {
	u8  message_size;
	u8  message_type;
	u16 memory_flags;
	u32 operation_id;
	u32 device_context;
	u32 block_id;
	u32 shared_mem_size;
	u32 shared_mem_start_offset;
	u32 shared_mem_descriptors;
	u32 padding[30];
};

struct  tf_tee_command_register_shared_memory_schannel {
	u8  message_size;
	u8  message_type;
	u16 memory_flags;
	u32 operation_id;
	u32 device_context;
	u32 block_id;
	u32 shared_mem_size;
	u32 shared_mem_start_offset;
	u8 shared_mem_descriptors[TF_TEE_MAX_COARSE_PAGES * 4];
};


/*
 * REGISTER_SHARED_MEMORY answer message.
 */
struct tf_tee_answer_register_shared_memory {
	u8                       message_size;
	u8                       message_type;
	u16                      message_info_rfu;
	/* an opaque Normal World identifier for the operation */
	u32                      operation_id;
	u32                      error_code;
	u32                      block;
};

/*
 * RELEASE_SHARED_MEMORY command message.
 */
struct tf_tee_command_release_shared_memory {
	u8                       message_size;
	u8                       message_type;
	u16                      message_info_rfu;
	/* an opaque Normal World identifier for the operation */
	u32                      operation_id;
	u32                      device_context;
	u32                      block;
};

/*
 * RELEASE_SHARED_MEMORY answer message.
 */
struct tf_tee_answer_release_shared_memory {
	u8                       message_size;
	u8                       message_type;
	u16                      message_info_rfu;
	u32                      operation_id;
	u32                      error_code;
	u32                      block_id;
};

struct tf_tee_command_invoke_client_command {
	u8                       message_size;
	u8                       message_type;
	u16                      param_types;
	u32                      operation_id;
	u32                      device_context;
	u32                      client_session;
	u64                      timeout;
	u32                      cancellation_id;
	u32                      client_command_identifier;
	union tf_tee_command_param   params[4];
};

/*
 * INVOKE_CLIENT_COMMAND command answer.
 */
struct tf_tee_answer_invoke_client_command {
	u8                     message_size;
	u8                     message_type;
	u8                     error_origin;
	u8                     __reserved;
	u32                    operation_id;
	u32                    error_code;
	union tf_tee_answer_param  answers[4];
};

/*
 * CANCEL_CLIENT_OPERATION command message.
 */
struct tf_tee_command_cancel_client_operation {
	u8                   message_size;
	u8                   message_type;
	u16                  message_info_rfu;
	/* an opaque Normal World identifier for the operation */
	u32                  operation_id;
	u32                  device_context;
	u32                  client_session;
	u32                  cancellation_id;
};

struct tf_tee_answer_cancel_client_operation {
	u8                   message_size;
	u8                   message_type;
	u16                  message_info_rfu;
	u32                  operation_id;
	u32                  error_code;
};

struct tf_tee_command_management {
	u8                   message_size;
	u8                   message_type;
	u16                  command;
	u32                  operation_id;
	u32                  w3b_size;
	u32                  w3b_start_offset;
	u32                  shared_mem_descriptors[1];
};

/*
 * Structure for L2 messages
 */
union tf_tee_command_128 {
	struct tf_tee_command_header                  header;
	struct tf_tee_command_create_device_context   create_device_context;
	struct tf_tee_command_destroy_device_context  destroy_device_context;
	struct tf_tee_command_open_client_session     open_client_session;
	struct tf_tee_command_close_client_session    close_client_session;
	struct tf_tee_command_register_shared_memory_128  register_shared_memory;
	struct tf_tee_command_release_shared_memory   release_shared_memory;
	struct tf_tee_command_invoke_client_command   invoke_client_command;
	struct tf_tee_command_cancel_client_operation cancel_client_operation;
	struct tf_tee_command_management              management;
};

union tf_tee_command_8 {
	struct tf_tee_command_header                    header;
	struct tf_tee_command_create_device_context     create_device_context;
	struct tf_tee_command_destroy_device_context    destroy_device_context;
	struct tf_tee_command_open_client_session       open_client_session;
	struct tf_tee_command_close_client_session      close_client_session;
	struct tf_tee_command_register_shared_memory_8  register_shared_memory;
	struct tf_tee_command_release_shared_memory     release_shared_memory;
	struct tf_tee_command_invoke_client_command     invoke_client_command;
	struct tf_tee_command_cancel_client_operation   cancel_client_operation;
	struct tf_tee_command_management                management;
};

union tf_tee_command_1 {
	struct tf_tee_command_header                    header;
	struct tf_tee_command_create_device_context     create_device_context;
	struct tf_tee_command_destroy_device_context    destroy_device_context;
	struct tf_tee_command_open_client_session       open_client_session;
	struct tf_tee_command_close_client_session      close_client_session;
	struct tf_tee_command_register_shared_memory_1  register_shared_memory;
	struct tf_tee_command_release_shared_memory     release_shared_memory;
	struct tf_tee_command_invoke_client_command     invoke_client_command;
	struct tf_tee_command_cancel_client_operation   cancel_client_operation;
	struct tf_tee_command_management                management;
};


union tf_tee_command {
	struct tf_tee_command_header                  header;
	struct tf_tee_command_create_device_context   create_device_context;
	struct tf_tee_command_destroy_device_context  destroy_device_context;
	struct tf_tee_command_open_client_session     open_client_session;
	struct tf_tee_command_close_client_session    close_client_session;
	struct tf_tee_command_register_shared_memory_schannel
		register_shared_memory;
	struct tf_tee_command_release_shared_memory   release_shared_memory;
	struct tf_tee_command_invoke_client_command   invoke_client_command;
	struct tf_tee_command_cancel_client_operation cancel_client_operation;
	struct tf_tee_command_management              management;
};


union tf_tee_answer {
	struct tf_tee_answer_header                  header;
	struct tf_tee_answer_create_device_context   create_device_context;
	struct tf_tee_answer_open_client_session     open_client_session;
	struct tf_tee_answer_close_client_session    close_client_session;
	struct tf_tee_answer_register_shared_memory  register_shared_memory;
	struct tf_tee_answer_release_shared_memory   release_shared_memory;
	struct tf_tee_answer_invoke_client_command   invoke_client_command;
	struct tf_tee_answer_destroy_device_context  destroy_device_context;
	struct tf_tee_answer_cancel_client_operation cancel_client_operation;
};

#endif
