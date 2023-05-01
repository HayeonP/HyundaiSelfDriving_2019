/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ISO14827-2"
 * 	found in "../j2735/J2735_201603DA+ITSK4-0.4_new.asn"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#include "Time.h"

static int
memb_deci_seconds_constraint_8(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 9)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_centi_seconds_constraint_8(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 99)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_milliseconds_constraint_8(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 999)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_timezone_Hour_qty_constraint_13(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= -13 && value <= 13)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_time_Minute_qty_constraint_13(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 59)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_time_Year_qty_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= -32768 && value <= 32767)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_time_Month_qty_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 1 && value <= 12)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_time_Day_qty_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 1 && value <= 31)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_time_Hour_qty_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 23)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_time_Minute_qty_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 59)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_time_Second_qty_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 60)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_memb_deci_seconds_constr_9 CC_NOTUSED = {
	{ 1, 1 }	/* (0..9) */,
	-1};
static asn_per_constraints_t asn_PER_memb_deci_seconds_constr_9 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  9 }	/* (0..9) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_centi_seconds_constr_10 CC_NOTUSED = {
	{ 1, 1 }	/* (0..99) */,
	-1};
static asn_per_constraints_t asn_PER_memb_centi_seconds_constr_10 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 7,  7,  0,  99 }	/* (0..99) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_milliseconds_constr_11 CC_NOTUSED = {
	{ 2, 1 }	/* (0..999) */,
	-1};
static asn_per_constraints_t asn_PER_memb_milliseconds_constr_11 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 10,  10,  0,  999 }	/* (0..999) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_time_SecondFractions_constr_8 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_time_SecondFractions_constr_8 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  0,  2 }	/* (0..2,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_timezone_Hour_qty_constr_14 CC_NOTUSED = {
	{ 1, 0 }	/* (-13..13) */,
	-1};
static asn_per_constraints_t asn_PER_memb_timezone_Hour_qty_constr_14 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 5,  5, -13,  13 }	/* (-13..13) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_time_Minute_qty_constr_15 CC_NOTUSED = {
	{ 1, 1 }	/* (0..59) */,
	-1};
static asn_per_constraints_t asn_PER_memb_time_Minute_qty_constr_15 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 6,  6,  0,  59 }	/* (0..59) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_time_Year_qty_constr_2 CC_NOTUSED = {
	{ 2, 0 }	/* (-32768..32767) */,
	-1};
static asn_per_constraints_t asn_PER_memb_time_Year_qty_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 16,  16, -32768,  32767 }	/* (-32768..32767) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_time_Month_qty_constr_3 CC_NOTUSED = {
	{ 1, 1 }	/* (1..12) */,
	-1};
static asn_per_constraints_t asn_PER_memb_time_Month_qty_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  1,  12 }	/* (1..12) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_time_Day_qty_constr_4 CC_NOTUSED = {
	{ 1, 1 }	/* (1..31) */,
	-1};
static asn_per_constraints_t asn_PER_memb_time_Day_qty_constr_4 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 5,  5,  1,  31 }	/* (1..31) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_time_Hour_qty_constr_5 CC_NOTUSED = {
	{ 1, 1 }	/* (0..23) */,
	-1};
static asn_per_constraints_t asn_PER_memb_time_Hour_qty_constr_5 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 5,  5,  0,  23 }	/* (0..23) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_time_Minute_qty_constr_6 CC_NOTUSED = {
	{ 1, 1 }	/* (0..59) */,
	-1};
static asn_per_constraints_t asn_PER_memb_time_Minute_qty_constr_6 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 6,  6,  0,  59 }	/* (0..59) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_time_Second_qty_constr_7 CC_NOTUSED = {
	{ 1, 1 }	/* (0..60) */,
	-1};
static asn_per_constraints_t asn_PER_memb_time_Second_qty_constr_7 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 6,  6,  0,  60 }	/* (0..60) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static int asn_DFL_5_cmp_0(const void *sptr) {
	const long *st = sptr;
	
	if(!st) {
		return -1; /* No value is not a default value */
	}
	
	/* Test default value 0 */
	return (*st != 0);
}
static int asn_DFL_5_set_0(void **sptr) {
	long *st = *sptr;
	
	if(!st) {
		st = (*sptr = CALLOC(1, sizeof(*st)));
		if(!st) return -1;
	}
	
	/* Install default value 0 */
	*st = 0;
	return 0;
}
static int asn_DFL_6_cmp_0(const void *sptr) {
	const long *st = sptr;
	
	if(!st) {
		return -1; /* No value is not a default value */
	}
	
	/* Test default value 0 */
	return (*st != 0);
}
static int asn_DFL_6_set_0(void **sptr) {
	long *st = *sptr;
	
	if(!st) {
		st = (*sptr = CALLOC(1, sizeof(*st)));
		if(!st) return -1;
	}
	
	/* Install default value 0 */
	*st = 0;
	return 0;
}
static int asn_DFL_7_cmp_0(const void *sptr) {
	const long *st = sptr;
	
	if(!st) {
		return -1; /* No value is not a default value */
	}
	
	/* Test default value 0 */
	return (*st != 0);
}
static int asn_DFL_7_set_0(void **sptr) {
	long *st = *sptr;
	
	if(!st) {
		st = (*sptr = CALLOC(1, sizeof(*st)));
		if(!st) return -1;
	}
	
	/* Install default value 0 */
	*st = 0;
	return 0;
}
static asn_TYPE_member_t asn_MBR_time_SecondFractions_8[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct Time__time_SecondFractions, deci_seconds),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_deci_seconds_constr_9, &asn_PER_memb_deci_seconds_constr_9,  memb_deci_seconds_constraint_8 },
		0, 0, /* No default value */
		"deci-seconds"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct Time__time_SecondFractions, centi_seconds),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_centi_seconds_constr_10, &asn_PER_memb_centi_seconds_constr_10,  memb_centi_seconds_constraint_8 },
		0, 0, /* No default value */
		"centi-seconds"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct Time__time_SecondFractions, milliseconds),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_milliseconds_constr_11, &asn_PER_memb_milliseconds_constr_11,  memb_milliseconds_constraint_8 },
		0, 0, /* No default value */
		"milliseconds"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_time_SecondFractions_tag2el_8[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* deci-seconds */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* centi-seconds */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* milliseconds */
};
static asn_CHOICE_specifics_t asn_SPC_time_SecondFractions_specs_8 = {
	sizeof(struct Time__time_SecondFractions),
	offsetof(struct Time__time_SecondFractions, _asn_ctx),
	offsetof(struct Time__time_SecondFractions, present),
	sizeof(((struct Time__time_SecondFractions *)0)->present),
	asn_MAP_time_SecondFractions_tag2el_8,
	3,	/* Count of tags in the map */
	0, 0,
	3	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_time_SecondFractions_8 = {
	"time-SecondFractions",
	"time-SecondFractions",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ &asn_OER_type_time_SecondFractions_constr_8, &asn_PER_type_time_SecondFractions_constr_8, CHOICE_constraint },
	asn_MBR_time_SecondFractions_8,
	3,	/* Elements count */
	&asn_SPC_time_SecondFractions_specs_8	/* Additional specs */
};

static int asn_DFL_14_cmp_0(const void *sptr) {
	const long *st = sptr;
	
	if(!st) {
		return -1; /* No value is not a default value */
	}
	
	/* Test default value 0 */
	return (*st != 0);
}
static int asn_DFL_14_set_0(void **sptr) {
	long *st = *sptr;
	
	if(!st) {
		st = (*sptr = CALLOC(1, sizeof(*st)));
		if(!st) return -1;
	}
	
	/* Install default value 0 */
	*st = 0;
	return 0;
}
static int asn_DFL_15_cmp_0(const void *sptr) {
	const long *st = sptr;
	
	if(!st) {
		return -1; /* No value is not a default value */
	}
	
	/* Test default value 0 */
	return (*st != 0);
}
static int asn_DFL_15_set_0(void **sptr) {
	long *st = *sptr;
	
	if(!st) {
		st = (*sptr = CALLOC(1, sizeof(*st)));
		if(!st) return -1;
	}
	
	/* Install default value 0 */
	*st = 0;
	return 0;
}
static asn_TYPE_member_t asn_MBR_time_Timezone_13[] = {
	{ ATF_NOFLAGS, 2, offsetof(struct Time__time_Timezone, timezone_Hour_qty),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_timezone_Hour_qty_constr_14, &asn_PER_memb_timezone_Hour_qty_constr_14,  memb_timezone_Hour_qty_constraint_13 },
		&asn_DFL_14_cmp_0,	/* Compare DEFAULT 0 */
		&asn_DFL_14_set_0,	/* Set DEFAULT 0 */
		"timezone-Hour-qty"
		},
	{ ATF_NOFLAGS, 1, offsetof(struct Time__time_Timezone, time_Minute_qty),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_time_Minute_qty_constr_15, &asn_PER_memb_time_Minute_qty_constr_15,  memb_time_Minute_qty_constraint_13 },
		&asn_DFL_15_cmp_0,	/* Compare DEFAULT 0 */
		&asn_DFL_15_set_0,	/* Set DEFAULT 0 */
		"time-Minute-qty"
		},
};
static const int asn_MAP_time_Timezone_oms_13[] = { 0, 1 };
static const ber_tlv_tag_t asn_DEF_time_Timezone_tags_13[] = {
	(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_time_Timezone_tag2el_13[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* timezone-Hour-qty */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* time-Minute-qty */
};
static asn_SEQUENCE_specifics_t asn_SPC_time_Timezone_specs_13 = {
	sizeof(struct Time__time_Timezone),
	offsetof(struct Time__time_Timezone, _asn_ctx),
	asn_MAP_time_Timezone_tag2el_13,
	2,	/* Count of tags in the map */
	asn_MAP_time_Timezone_oms_13,	/* Optional members */
	2, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_time_Timezone_13 = {
	"time-Timezone",
	"time-Timezone",
	&asn_OP_SEQUENCE,
	asn_DEF_time_Timezone_tags_13,
	sizeof(asn_DEF_time_Timezone_tags_13)
		/sizeof(asn_DEF_time_Timezone_tags_13[0]) - 1, /* 1 */
	asn_DEF_time_Timezone_tags_13,	/* Same as above */
	sizeof(asn_DEF_time_Timezone_tags_13)
		/sizeof(asn_DEF_time_Timezone_tags_13[0]), /* 2 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_time_Timezone_13,
	2,	/* Elements count */
	&asn_SPC_time_Timezone_specs_13	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_Time_1[] = {
	{ ATF_POINTER, 6, offsetof(struct Time, time_Year_qty),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_time_Year_qty_constr_2, &asn_PER_memb_time_Year_qty_constr_2,  memb_time_Year_qty_constraint_1 },
		0, 0, /* No default value */
		"time-Year-qty"
		},
	{ ATF_POINTER, 5, offsetof(struct Time, time_Month_qty),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_time_Month_qty_constr_3, &asn_PER_memb_time_Month_qty_constr_3,  memb_time_Month_qty_constraint_1 },
		0, 0, /* No default value */
		"time-Month-qty"
		},
	{ ATF_POINTER, 4, offsetof(struct Time, time_Day_qty),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_time_Day_qty_constr_4, &asn_PER_memb_time_Day_qty_constr_4,  memb_time_Day_qty_constraint_1 },
		0, 0, /* No default value */
		"time-Day-qty"
		},
	{ ATF_NOFLAGS, 3, offsetof(struct Time, time_Hour_qty),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_time_Hour_qty_constr_5, &asn_PER_memb_time_Hour_qty_constr_5,  memb_time_Hour_qty_constraint_1 },
		&asn_DFL_5_cmp_0,	/* Compare DEFAULT 0 */
		&asn_DFL_5_set_0,	/* Set DEFAULT 0 */
		"time-Hour-qty"
		},
	{ ATF_NOFLAGS, 2, offsetof(struct Time, time_Minute_qty),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_time_Minute_qty_constr_6, &asn_PER_memb_time_Minute_qty_constr_6,  memb_time_Minute_qty_constraint_1 },
		&asn_DFL_6_cmp_0,	/* Compare DEFAULT 0 */
		&asn_DFL_6_set_0,	/* Set DEFAULT 0 */
		"time-Minute-qty"
		},
	{ ATF_NOFLAGS, 1, offsetof(struct Time, time_Second_qty),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_time_Second_qty_constr_7, &asn_PER_memb_time_Second_qty_constr_7,  memb_time_Second_qty_constraint_1 },
		&asn_DFL_7_cmp_0,	/* Compare DEFAULT 0 */
		&asn_DFL_7_set_0,	/* Set DEFAULT 0 */
		"time-Second-qty"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct Time, time_SecondFractions),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_time_SecondFractions_8,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"time-SecondFractions"
		},
	{ ATF_POINTER, 1, offsetof(struct Time, time_Timezone),
		(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
		0,
		&asn_DEF_time_Timezone_13,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"time-Timezone"
		},
};
static const int asn_MAP_Time_oms_1[] = { 0, 1, 2, 3, 4, 5, 7 };
static const ber_tlv_tag_t asn_DEF_Time_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_Time_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* time-Year-qty */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* time-Month-qty */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* time-Day-qty */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* time-Hour-qty */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* time-Minute-qty */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* time-Second-qty */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 }, /* time-SecondFractions */
    { (ASN_TAG_CLASS_CONTEXT | (7 << 2)), 7, 0, 0 } /* time-Timezone */
};
asn_SEQUENCE_specifics_t asn_SPC_Time_specs_1 = {
	sizeof(struct Time),
	offsetof(struct Time, _asn_ctx),
	asn_MAP_Time_tag2el_1,
	8,	/* Count of tags in the map */
	asn_MAP_Time_oms_1,	/* Optional members */
	7, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_Time = {
	"Time",
	"Time",
	&asn_OP_SEQUENCE,
	asn_DEF_Time_tags_1,
	sizeof(asn_DEF_Time_tags_1)
		/sizeof(asn_DEF_Time_tags_1[0]), /* 1 */
	asn_DEF_Time_tags_1,	/* Same as above */
	sizeof(asn_DEF_Time_tags_1)
		/sizeof(asn_DEF_Time_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_Time_1,
	8,	/* Elements count */
	&asn_SPC_Time_specs_1	/* Additional specs */
};
