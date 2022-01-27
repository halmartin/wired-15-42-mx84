#ifndef _BIRD_MERAKI_ROUTE_H_
#define _BIRD_MERAKI_ROUTE_H_

#include "nest/route.h"
#include "nest/attrs.h"

#define ASPATH_BUF_LEN 512
// Accommodate worst-case nexthop size
#define NEXTHOP_BUF_LEN sizeof("[\"xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx\", "\
                               "\"xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx\"]")
// Accommodate worst case bgp source_key size
#define BGP_SOURCE_KEY_LEN sizeof("BGP:xBGP:4294967295:xxx.xxx.xxx.xxx")

// Accommodate worst case dotted decimal representation of an IPv4 address
// with null-terminating character
#define DOTTED_DECIMAL_IPV4_ADDR_LEN    (STD_ADDRESS_P_LENGTH + 1)

char get_bgp_origin_char(const eattr *origin_attr);
void get_bgp_source_key(rte *e, char* source_key, int len, char bgp_type);
void export_route_state_json(struct rt_show_data *);

#endif
