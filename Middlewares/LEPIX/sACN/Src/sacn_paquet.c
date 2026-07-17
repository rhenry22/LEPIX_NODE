/* Offsets fixes dans le paquet E1.31 */
#define SACN_PKT_SIZE(slots)   (126 + (slots))

/* Root Layer PDU */
#define OFF_PREAMBLE           0    /* 0x0010 */
#define OFF_POSTAMBLE          2    /* 0x0000 */
#define OFF_IDENT              4    /* "ASC-E1.17\0\0\0\0" */
#define OFF_FLAGS_ROOT         16
#define OFF_VECTOR_ROOT        18   /* 0x00000004 */
#define OFF_CID                22   /* 16 octets */

/* Framing Layer */
#define OFF_FLAGS_FRAME        38
#define OFF_VECTOR_FRAME       40   /* 0x00000002 */
#define OFF_SOURCE_NAME        44   /* 64 octets */
#define OFF_PRIORITY          108
#define OFF_SYNC_ADDR         109
#define OFF_SEQ_NUM           111
#define OFF_OPTIONS           112
#define OFF_UNIVERSE          113

/* DMP Layer */
#define OFF_FLAGS_DMP         115
#define OFF_VECTOR_DMP        117   /* 0x02 */
#define OFF_ADDR_TYPE         118   /* 0xa1 */
#define OFF_FIRST_PROP        119   /* 0x0000 */
#define OFF_ADDR_INC          121   /* 0x0001 */
#define OFF_PROP_COUNT        123   /* slots + 1 (startcode) */
#define OFF_START_CODE        125   /* 0x00 (DMX) */
#define OFF_DMX_DATA          126