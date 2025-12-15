#ifndef __UTIL_H__
#define __UTIL_H__

void comm_session(bool start_stop);
void emergency_stop(void);
void hv_iso_test_enable(bool enable, uint32_t voltage);

#endif /* __UTIL_H__ */