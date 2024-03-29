/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ISO14827-2"
 * 	found in "../j2735/J2735_201603DA+ITSK4-0.4_new.asn"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#include "Logout.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static asn_oer_constraints_t asn_OER_type_Logout_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
asn_per_constraints_t asn_PER_type_Logout_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  3,  3,  0,  6 }	/* (0..6,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_Logout_value2enum_1[] = {
	{ 0,	5,	"other" },
	{ 1,	15,	"serverRequested" },
	{ 2,	15,	"clientRequested" },
	{ 3,	14,	"serverShutdown" },
	{ 4,	14,	"clientShutdown" },
	{ 5,	18,	"serverCommProblems" },
	{ 6,	18,	"clientCommProblems" }
	/* This list is extensible */
};
static const unsigned int asn_MAP_Logout_enum2value_1[] = {
	6,	/* clientCommProblems(6) */
	2,	/* clientRequested(2) */
	4,	/* clientShutdown(4) */
	0,	/* other(0) */
	5,	/* serverCommProblems(5) */
	1,	/* serverRequested(1) */
	3	/* serverShutdown(3) */
	/* This list is extensible */
};
const asn_INTEGER_specifics_t asn_SPC_Logout_specs_1 = {
	asn_MAP_Logout_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_Logout_enum2value_1,	/* N => "tag"; sorted by N */
	7,	/* Number of elements in the maps */
	8,	/* Extensions before this member */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_Logout_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_Logout = {
	"Logout",
	"Logout",
	&asn_OP_NativeEnumerated,
	asn_DEF_Logout_tags_1,
	sizeof(asn_DEF_Logout_tags_1)
		/sizeof(asn_DEF_Logout_tags_1[0]), /* 1 */
	asn_DEF_Logout_tags_1,	/* Same as above */
	sizeof(asn_DEF_Logout_tags_1)
		/sizeof(asn_DEF_Logout_tags_1[0]), /* 1 */
	{ &asn_OER_type_Logout_constr_1, &asn_PER_type_Logout_constr_1, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_Logout_specs_1	/* Additional specs */
};

