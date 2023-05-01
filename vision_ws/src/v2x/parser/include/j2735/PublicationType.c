/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ISO14827-2"
 * 	found in "../j2735/J2735_201603DA+ITSK4-0.4_new.asn"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#include "PublicationType.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static asn_oer_constraints_t asn_OER_type_datexPublication_Management_cd_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_datexPublication_Management_cd_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  4,  4,  0,  9 }	/* (0..9,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_PublicationType_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
asn_per_constraints_t asn_PER_type_PublicationType_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 1,  1,  0,  1 }	/* (0..1) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_datexPublication_Management_cd_value2enum_2[] = {
	{ 0,	20,	"temporarilySuspended" },
	{ 1,	6,	"resume" },
	{ 2,	15,	"terminate-other" },
	{ 3,	31,	"terminate-dataNoLongerAvailable" },
	{ 4,	35,	"terminate-publicationsBeingRejected" },
	{ 5,	25,	"terminate-PendingShutdown" },
	{ 6,	24,	"terminate-processingMgmt" },
	{ 7,	23,	"terminate-bandwidthMgmt" },
	{ 8,	22,	"terminate-accessDenied" },
	{ 9,	14,	"unknownRequest" }
	/* This list is extensible */
};
static const unsigned int asn_MAP_datexPublication_Management_cd_enum2value_2[] = {
	1,	/* resume(1) */
	0,	/* temporarilySuspended(0) */
	5,	/* terminate-PendingShutdown(5) */
	8,	/* terminate-accessDenied(8) */
	7,	/* terminate-bandwidthMgmt(7) */
	3,	/* terminate-dataNoLongerAvailable(3) */
	2,	/* terminate-other(2) */
	6,	/* terminate-processingMgmt(6) */
	4,	/* terminate-publicationsBeingRejected(4) */
	9	/* unknownRequest(9) */
	/* This list is extensible */
};
static const asn_INTEGER_specifics_t asn_SPC_datexPublication_Management_cd_specs_2 = {
	asn_MAP_datexPublication_Management_cd_value2enum_2,	/* "tag" => N; sorted by tag */
	asn_MAP_datexPublication_Management_cd_enum2value_2,	/* N => "tag"; sorted by N */
	10,	/* Number of elements in the maps */
	11,	/* Extensions before this member */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_datexPublication_Management_cd_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_datexPublication_Management_cd_2 = {
	"datexPublication-Management-cd",
	"datexPublication-Management-cd",
	&asn_OP_NativeEnumerated,
	asn_DEF_datexPublication_Management_cd_tags_2,
	sizeof(asn_DEF_datexPublication_Management_cd_tags_2)
		/sizeof(asn_DEF_datexPublication_Management_cd_tags_2[0]) - 1, /* 1 */
	asn_DEF_datexPublication_Management_cd_tags_2,	/* Same as above */
	sizeof(asn_DEF_datexPublication_Management_cd_tags_2)
		/sizeof(asn_DEF_datexPublication_Management_cd_tags_2[0]), /* 2 */
	{ &asn_OER_type_datexPublication_Management_cd_constr_2, &asn_PER_type_datexPublication_Management_cd_constr_2, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_datexPublication_Management_cd_specs_2	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_PublicationType_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct PublicationType, datexPublication_Management_cd),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_datexPublication_Management_cd_2,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"datexPublication-Management-cd"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PublicationType, datexPublish_Data),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MessageFrame,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"datexPublish-Data"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_PublicationType_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* datexPublication-Management-cd */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* datexPublish-Data */
};
asn_CHOICE_specifics_t asn_SPC_PublicationType_specs_1 = {
	sizeof(struct PublicationType),
	offsetof(struct PublicationType, _asn_ctx),
	offsetof(struct PublicationType, present),
	sizeof(((struct PublicationType *)0)->present),
	asn_MAP_PublicationType_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0,
	-1	/* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_PublicationType = {
	"PublicationType",
	"PublicationType",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ &asn_OER_type_PublicationType_constr_1, &asn_PER_type_PublicationType_constr_1, CHOICE_constraint },
	asn_MBR_PublicationType_1,
	2,	/* Elements count */
	&asn_SPC_PublicationType_specs_1	/* Additional specs */
};
