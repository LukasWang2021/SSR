/**
 * @file proto_define.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-29
 */
#ifndef _PROTO_DEFINE_H_
#define _PROTO_DEFINE_H_

/**
 * @brief 
 *
 * @param obj: field object
 * @param field: field to find 
 * @param field_buffer:recieved buffer from TP
 * @param field_size: size of the field
 * 
 */
#define PARSE_FIELD(obj, field, field_buffer, field_size, ret)\
		do\
		{\
			pb_istream_t stream = {0};\
			stream = pb_istream_from_buffer(field_buffer, field_size);\
			ret = pb_decode(&stream, field##_fields, &obj);\
		}\
		while(0)

/**
 * @brief 
 *
 * @param obj: field object
 * @param field: field to find 
 * @param field_buffer:buffer to send to TP
 * @param buffer_size: size of the buffer
 * @param bytes_written: bytes that have been written
 *
 */

#define SET_FIELD(obj, field, field_buffer, buffer_size, hash_size, bytes, ret)\
		do\
		{\
			memcpy(field_buffer, get_hash<field>(), hash_size);\
			pb_ostream_t stream = {0};\
			stream = pb_ostream_from_buffer(field_buffer+hash_size, buffer_size-hash_size);\
			ret = pb_encode(&stream, field##_fields, &obj);\
			bytes = stream.bytes_written+hash_size;\
		}\
		while(0)


#define HASH_CMP(field, buffer)	compare_int(get_hash<BaseTypes_##field>(), buffer)

#endif
