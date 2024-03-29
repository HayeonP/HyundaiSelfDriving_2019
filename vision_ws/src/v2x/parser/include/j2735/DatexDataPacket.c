/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ISO14827-2"
 * 	found in "../j2735/J2735_201603DA+ITSK4-0.4_new.asn"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#include "DatexDataPacket.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_datex_Crc_nbr_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const OCTET_STRING_t *st = (const OCTET_STRING_t *)sptr;
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	size = st->size;
	
	if((size == 2)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_type_datex_Version_number_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_datex_Version_number_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  1,  1,  0,  1 }	/* (0..1,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_datex_Crc_nbr_constr_7 CC_NOTUSED = {
	{ 0, 0 },
	2	/* (SIZE(2..2)) */};
static asn_per_constraints_t asn_PER_memb_datex_Crc_nbr_constr_7 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 0,  0,  2,  2 }	/* (SIZE(2..2)) */,
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_datex_Version_number_value2enum_2[] = {
	{ 0,	12,	"experimental" },
	{ 1,	8,	"version1" }
	/* This list is extensible */
};
static const unsigned int asn_MAP_datex_Version_number_enum2value_2[] = {
	0,	/* experimental(0) */
	1	/* version1(1) */
	/* This list is extensible */
};
static const asn_INTEGER_specifics_t asn_SPC_datex_Version_number_specs_2 = {
	asn_MAP_datex_Version_number_value2enum_2,	/* "tag" => N; sorted by tag */
	asn_MAP_datex_Version_number_enum2value_2,	/* N => "tag"; sorted by N */
	2,	/* Number of elements in the maps */
	3,	/* Extensions before this member */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_datex_Version_number_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_datex_Version_number_2 = {
	"datex-Version-number",
	"datex-Version-number",
	&asn_OP_NativeEnumerated,
	asn_DEF_datex_Version_number_tags_2,
	sizeof(asn_DEF_datex_Version_number_tags_2)
		/sizeof(asn_DEF_datex_Version_number_tags_2[0]) - 1, /* 1 */
	asn_DEF_datex_Version_number_tags_2,	/* Same as above */
	sizeof(asn_DEF_datex_Version_number_tags_2)
		/sizeof(asn_DEF_datex_Version_number_tags_2[0]), /* 2 */
	{ &asn_OER_type_datex_Version_number_constr_2, &asn_PER_type_datex_Version_number_constr_2, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_datex_Version_number_specs_2	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_DatexDataPacket_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct DatexDataPacket, datex_Version_number),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_datex_Version_number_2,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"datex-Version-number"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct DatexDataPacket, datex_Data),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"datex-Data"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct DatexDataPacket, datex_Crc_nbr),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		0,
		{ &asn_OER_memb_datex_Crc_nbr_constr_7, &asn_PER_memb_datex_Crc_nbr_constr_7,  memb_datex_Crc_nbr_constraint_1 },
		0, 0, /* No default value */
		"datex-Crc-nbr"
		},
};
static const ber_tlv_tag_t asn_DEF_DatexDataPacket_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_DatexDataPacket_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* datex-Version-number */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* datex-Data */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* datex-Crc-nbr */
};
static asn_SEQUENCE_specifics_t asn_SPC_DatexDataPacket_specs_1 = {
	sizeof(struct DatexDataPacket),
	offsetof(struct DatexDataPacket, _asn_ctx),
	asn_MAP_DatexDataPacket_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_DatexDataPacket = {
	"DatexDataPacket",
	"DatexDataPacket",
	&asn_OP_SEQUENCE,
	asn_DEF_DatexDataPacket_tags_1,
	sizeof(asn_DEF_DatexDataPacket_tags_1)
		/sizeof(asn_DEF_DatexDataPacket_tags_1[0]), /* 1 */
	asn_DEF_DatexDataPacket_tags_1,	/* Same as above */
	sizeof(asn_DEF_DatexDataPacket_tags_1)
		/sizeof(asn_DEF_DatexDataPacket_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_DatexDataPacket_1,
	3,	/* Elements count */
	&asn_SPC_DatexDataPacket_specs_1	/* Additional specs */
};

