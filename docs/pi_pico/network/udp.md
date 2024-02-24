# UDP

`pico-sdk/lib/lwip/src/include/lwip/ip_addr.h`

```c
#if LWIP_IPV4 && LWIP_IPV6
/**
 * @ingroup ipaddr
 * A union struct for both IP version's addresses.
 * ATTENTION: watch out for its size when adding IPv6 address scope!
 */
typedef struct ip_addr {
  union {
    ip6_addr_t ip6;
    ip4_addr_t ip4;
  } u_addr;

  /** @ref lwip_ip_addr_type */
  u8_t type;
} ip_addr_t;
```

```c
/** This is the aligned version of ip4_addr_t,
   used as local variable, on the stack, etc. */
struct ip4_addr {
  u32_t addr;
};

/** ip4_addr_t uses a struct for convenience only, so that the same defines can
 * operate both on ip4_addr_t as well as on ip4_addr_p_t. */
typedef struct ip4_addr ip4_addr_t;
```

```c
/** This is the aligned version of ip6_addr_t,
    used as local variable, on the stack, etc. */
struct ip6_addr {
  u32_t addr[4];
#if LWIP_IPV6_SCOPES
  u8_t zone;
#endif /* LWIP_IPV6_SCOPES */
};

/** IPv6 address */
typedef struct ip6_addr ip6_addr_t;
```