/**
  ******************************************************************************
  * @file    communication/ethernet/ethernet.cpp
  * @brief   FCU Ethernet driver: raw Ethernet/IPv4/UDP/ICMP/ARP stack on the
  *          STM32H7 HAL ETH peripheral, the DMA RX pool and its interrupt
  *          callbacks, and the platform-side definition of the logic UDP
  *          interface (communication/interfaces/ethernet.hpp).
  ******************************************************************************
  */

#include "communication/interfaces/ethernet.hpp"   // logic::communication::udp seam
#include "communication/ethernet/ethernet.hpp"      // platform init/process

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>

#include "stm32h7xx_hal.h"
#include "eth.h"   // extern "C" ETH_HandleTypeDef heth;

extern "C" ETH_TxPacketConfig TxConfig;   // defined in eth.c

using logic::communication::Datagram;
using logic::communication::Endpoint;
using logic::communication::MAC_LENGTH_BYTES;
using logic::communication::NetError;

namespace {

/* The link's telemetry record. The board has one Ethernet, so this is file-
   static like the rest of the stack state; the Ethernet handle exposes it
   through info(). Updated from CPU context (init / send / the tick() RX drain),
   never from the DMA ISR. */
EthernetInfo s_info{};

/* ---- Byte-order helpers -------------------------------------------------- */
inline uint16_t hton16(uint16_t v) { return __builtin_bswap16(v); }
inline uint16_t ntoh16(uint16_t v) { return __builtin_bswap16(v); }
inline uint32_t hton32(uint32_t v) { return __builtin_bswap32(v); }
inline uint32_t ntoh32(uint32_t v) { return __builtin_bswap32(v); }

constexpr uint32_t make_ipv4(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
    return (static_cast<uint32_t>(b1) << 24) | (static_cast<uint32_t>(b2) << 16) |
           (static_cast<uint32_t>(b3) << 8) | static_cast<uint32_t>(b4);
}

/* ---- Protocol constants -------------------------------------------------- */
constexpr uint16_t ETHERTYPE_IPV4 = 0x0800;
constexpr uint16_t ETHERTYPE_ARP  = 0x0806;

constexpr uint16_t ETH_MAX_PAYLOAD_LEN_BYTES = 1500;
constexpr uint16_t IPV4_MAX_DATA_LEN_BYTES   = 1440;
constexpr uint8_t  IPV4_PROTOCOL_ICMP = 1;
constexpr uint8_t  IPV4_PROTOCOL_UDP  = 17;

constexpr uint16_t IPV4_DF_FLAG     = 0x4000;
constexpr uint16_t IPV4_MF_FLAG     = 0x2000;
constexpr uint16_t IPV4_OFFSET_MASK = 0x1FFF;
constexpr uint8_t  IPV4_DEFAULT_TTL = 64;

constexpr uint8_t  ICMP_TYPE_ECHO_REPLY = 0;
constexpr uint8_t  ICMP_TYPE_ECHO       = 8;

constexpr uint16_t ARP_HTYPE_ETHERNET = 1;
constexpr uint16_t ARP_PTYPE_IPV4     = 0x0800;
constexpr uint8_t  ARP_HLEN_ETHERNET  = 6;
constexpr uint8_t  ARP_PLEN_IPV4      = 4;
constexpr uint16_t ARP_OPER_REQUEST   = 1;
constexpr uint16_t ARP_OPER_REPLY     = 2;

/* ---- Local host addressing ----------------------------------------------- */
constexpr uint32_t LOCAL_IP        = make_ipv4(192, 168, 0, 100);
constexpr uint16_t LOCAL_PORT      = 55555;

/* ---- RX DMA pool --------------------------------------------------------- */
constexpr std::size_t RX_BUF_SIZE_BYTES = 1536;
/* MUST stay strictly greater than ETH_RX_DESC_CNT (4). When the RX ISR pops a frame it
   immediately re-arms that descriptor via HAL_ETH_RxAllocateCallback (the only path that
   re-arms RX descriptors). With as many buffers as descriptors there is never a spare buffer
   at that instant, the callback returns null, the descriptor is left dead, and after the DMA
   chews through its descriptors RX stops for good (telemetry TX keeps running, masking it).
   The spare buffers here guarantee the re-arm always finds one — even when a whole halt's worth
   of frames (the debugger case) is drained in one ISR pass. 12 buffers = 4 descriptors + 8
   spare, ~18 KB in the 288 KB Ethernet DMA SRAM region. */
constexpr std::size_t RX_BUF_COUNT      = 12;

/* ---- RX datagram ring for the UDP seam ----------------------------------- */
constexpr std::size_t MAX_UDP_PAYLOAD_BYTES      = IPV4_MAX_DATA_LEN_BYTES - 8;  // minus UDP header
constexpr std::size_t RX_RING_CAPACITY_DATAGRAMS = 4;

enum class BufStatus { Free, OwnedCpu, OwnedDma };

/* ---- Packet layouts ------------------------------------------------------ */
struct __attribute__((packed)) EthHeader {
    uint8_t  dst[6];
    uint8_t  src[6];
    uint16_t ethertype;
};

struct __attribute__((packed)) Ipv4Header {
    uint8_t  ihl : 4;
    uint8_t  version : 4;
    uint8_t  ecn : 2;
    uint8_t  dscp : 6;
    uint16_t len;
    uint16_t id;
    uint16_t frag;
    uint8_t  ttl;
    uint8_t  protocol;
    uint16_t checksum;
    uint32_t src;
    uint32_t dst;
};

struct __attribute__((packed)) IcmpEchoHeader {
    uint8_t  type;
    uint8_t  code;
    uint16_t checksum;
    uint16_t id;
    uint16_t seq;
};

struct __attribute__((packed)) UdpHeader {
    uint16_t src_port;
    uint16_t dst_port;
    uint16_t len;
    uint16_t checksum;
};

struct __attribute__((packed)) ArpPacket {
    uint16_t htype;
    uint16_t ptype;
    uint8_t  hlen;
    uint8_t  plen;
    uint16_t oper;
    uint8_t  sha[6];
    uint32_t spa;
    uint8_t  tha[6];
    uint32_t tpa;
};

/* ---- RX pool state (shared with the HAL DMA callbacks below) -------------- */
uint8_t   rx_pool[RX_BUF_COUNT][RX_BUF_SIZE_BYTES] __attribute__((section(".RxBuffSection")));
BufStatus rx_buf_status[RX_BUF_COUNT];
volatile uint8_t rx_queue_size = 0;

/* ---- RX datagram ring ---------------------------------------------------- */
struct RxSlot {
    Endpoint    source;
    std::array<uint8_t, MAX_UDP_PAYLOAD_BYTES> data{};
    std::size_t length_bytes = 0;
};

std::array<RxSlot, RX_RING_CAPACITY_DATAGRAMS> rx_ring{};
std::size_t rx_ring_head = 0;
std::size_t rx_ring_tail = 0;

/* ---- TX frame templates (scatter-gather buffers in the DMA TX region) ----- */
ETH_BufferTypeDef tx_eth_hdr_buf;
EthHeader         tx_eth_hdr __attribute__((section(".TxBuffSection")));

ETH_BufferTypeDef tx_ipv4_hdr_buf;
Ipv4Header        tx_ipv4_hdr __attribute__((section(".TxBuffSection")));

ETH_BufferTypeDef tx_icmp_reply_buf;
uint8_t           tx_icmp_reply[IPV4_MAX_DATA_LEN_BYTES] __attribute__((section(".TxBuffSection")));

ETH_BufferTypeDef tx_udp_hdr_buf;
UdpHeader         tx_udp_hdr __attribute__((section(".TxBuffSection")));

ETH_BufferTypeDef tx_udp_data_buf;
uint8_t           tx_udp_data[IPV4_MAX_DATA_LEN_BYTES] __attribute__((section(".TxBuffSection")));

ETH_BufferTypeDef tx_arp_buf;
ArpPacket         tx_arp __attribute__((section(".TxBuffSection")));

/* ---- Header preparation -------------------------------------------------- */
void prepare_eth_header(const uint8_t dst[6], uint16_t ethertype)
{
    std::memcpy(tx_eth_hdr.dst, dst, 6);
    tx_eth_hdr.ethertype = hton16(ethertype);
}

void prepare_ipv4_header(uint16_t data_len_bytes, uint8_t protocol, uint32_t dst_ip)
{
    tx_ipv4_hdr.len      = hton16(sizeof(Ipv4Header) + data_len_bytes);
    tx_ipv4_hdr.protocol = protocol;
    tx_ipv4_hdr.dst      = hton32(dst_ip);
    tx_eth_hdr_buf.next  = &tx_ipv4_hdr_buf;
}

void prepare_udp_header(uint16_t data_len_bytes, uint16_t dst_port)
{
    tx_udp_hdr.dst_port = hton16(dst_port);
    tx_udp_hdr.len      = hton16(sizeof(UdpHeader) + data_len_bytes);
    tx_udp_data_buf.len = data_len_bytes;
    tx_ipv4_hdr_buf.next = &tx_udp_hdr_buf;
}

void prepare_arp_header(const uint8_t mac[6], uint32_t ip, uint16_t oper)
{
    tx_arp.oper = hton16(oper);
    std::memcpy(tx_arp.tha, mac, 6);
    tx_arp.tpa = hton32(ip);
    tx_eth_hdr_buf.next = &tx_arp_buf;
}

/* ---- RX dispatch --------------------------------------------------------- */
void enqueue_rx(const Endpoint& source, const uint8_t* payload, std::size_t length_bytes)
{
    if (length_bytes > MAX_UDP_PAYLOAD_BYTES) {
        length_bytes = MAX_UDP_PAYLOAD_BYTES;
    }

    RxSlot& slot = rx_ring[rx_ring_head];
    slot.source = source;
    std::memcpy(slot.data.data(), payload, length_bytes);
    slot.length_bytes = length_bytes;

    rx_ring_head = (rx_ring_head + 1) % RX_RING_CAPACITY_DATAGRAMS;
    if (rx_ring_head == rx_ring_tail) {
        rx_ring_tail = (rx_ring_tail + 1) % RX_RING_CAPACITY_DATAGRAMS;  // drop oldest
        if (s_info.rx_dropped != 0xFFFFu) {
            ++s_info.rx_dropped;  // saturating drop counter (telemetry)
        }
    }
}

void process_udp(Endpoint& source, const uint8_t* udp_buf)
{
    const auto* rx_udp = reinterpret_cast<const UdpHeader*>(udp_buf);
    const uint16_t len = ntoh16(rx_udp->len);

    if (len < sizeof(UdpHeader) || len > IPV4_MAX_DATA_LEN_BYTES) {
        return;
    }
    if (ntoh16(rx_udp->dst_port) != LOCAL_PORT) {
        return;  // no application on this port
    }

    source.port = ntoh16(rx_udp->src_port);
    enqueue_rx(source, udp_buf + sizeof(UdpHeader), len - sizeof(UdpHeader));
}

void process_icmp_echo_request(const Endpoint& source, const uint8_t* icmp_buf, uint16_t icmp_len)
{
    const auto* rx_icmp = reinterpret_cast<const IcmpEchoHeader*>(icmp_buf);
    if (rx_icmp->code != 0) {
        return;
    }

    prepare_eth_header(source.mac.data(), ETHERTYPE_IPV4);
    prepare_ipv4_header(icmp_len, IPV4_PROTOCOL_ICMP, source.ipv4);
    tx_ipv4_hdr_buf.next = &tx_icmp_reply_buf;
    tx_icmp_reply_buf.len = icmp_len;

    std::memcpy(tx_icmp_reply, icmp_buf, icmp_len);
    reinterpret_cast<IcmpEchoHeader*>(tx_icmp_reply)->type = ICMP_TYPE_ECHO_REPLY;

    TxConfig.Length = sizeof(EthHeader) + sizeof(Ipv4Header) + icmp_len;
    HAL_ETH_Transmit_IT(&heth, &TxConfig);
}

void process_icmp(const Endpoint& source, const uint8_t* icmp_buf, uint16_t icmp_len)
{
    const auto* rx_icmp = reinterpret_cast<const IcmpEchoHeader*>(icmp_buf);
    if (rx_icmp->type == ICMP_TYPE_ECHO) {
        process_icmp_echo_request(source, icmp_buf, icmp_len);
    }
}

void process_ipv4(Endpoint& source, const uint8_t* ipv4_buf)
{
    const auto* rx_ipv4 = reinterpret_cast<const Ipv4Header*>(ipv4_buf);

    if (rx_ipv4->ihl > 5) {
        return;  // options unsupported
    }
    if (ntoh32(rx_ipv4->dst) != LOCAL_IP) {
        return;  // not for us
    }

    const uint16_t len = ntoh16(rx_ipv4->len);
    if (len < sizeof(Ipv4Header) || len > ETH_MAX_PAYLOAD_LEN_BYTES) {
        return;
    }

    const uint16_t frag = ntoh16(rx_ipv4->frag);
    if ((frag & IPV4_MF_FLAG) || (frag & IPV4_OFFSET_MASK)) {
        return;  // fragmentation unsupported
    }

    source.ipv4 = ntoh32(rx_ipv4->src);

    const uint8_t* data    = ipv4_buf + sizeof(Ipv4Header);
    const uint16_t data_len = len - sizeof(Ipv4Header);

    switch (rx_ipv4->protocol) {
        case IPV4_PROTOCOL_ICMP: process_icmp(source, data, data_len); break;
        case IPV4_PROTOCOL_UDP:  process_udp(source, data);            break;
        default: break;
    }
}

void send_arp_reply(const uint8_t mac[6], uint32_t ip)
{
    prepare_eth_header(mac, ETHERTYPE_ARP);
    prepare_arp_header(mac, ip, ARP_OPER_REPLY);
    TxConfig.Length = sizeof(EthHeader) + sizeof(ArpPacket);
    HAL_ETH_Transmit_IT(&heth, &TxConfig);
}

void process_arp(const uint8_t* arp_buf)
{
    const auto* rx_arp = reinterpret_cast<const ArpPacket*>(arp_buf);
    if (ntoh32(rx_arp->tpa) != LOCAL_IP || ntoh16(rx_arp->oper) != ARP_OPER_REQUEST) {
        return;
    }
    send_arp_reply(rx_arp->sha, ntoh32(rx_arp->spa));
}

void process_eth_frame(const uint8_t* frame)
{
    const auto* hdr = reinterpret_cast<const EthHeader*>(frame);
    const uint16_t ethertype = ntoh16(hdr->ethertype);
    const uint8_t* payload = frame + sizeof(EthHeader);

    Endpoint source;
    std::memcpy(source.mac.data(), hdr->src, MAC_LENGTH_BYTES);

    switch (ethertype) {
        case ETHERTYPE_IPV4: process_ipv4(source, payload); break;
        case ETHERTYPE_ARP:  process_arp(payload);          break;
        default: break;
    }
}

/* -------------------------------------------------------------------------- */
/* PHY (LAN8742) bring-up over MDIO                                           */
/*                                                                            */
/* CubeMX's MX_ETH_Init configures the MAC/DMA but NEVER touches the PHY, so   */
/* link establishment was left entirely to the PHY's reset straps and the MAC */
/* ran at HAL_ETH_Init's fixed default speed. That linked to a PC on a direct  */
/* cable (parallel detection) but not through an auto-negotiating switch. Here */
/* we force auto-negotiation on, wait (bounded) for the link, and match the    */
/* MAC speed/duplex to whatever was negotiated.                                */
/* -------------------------------------------------------------------------- */

constexpr uint16_t PHY_BCR        = 0x0000;  // Basic Control Register
constexpr uint16_t PHY_BSR        = 0x0001;  // Basic Status Register
constexpr uint16_t PHY_PHYSID1    = 0x0002;  // PHY identifier 1 (used to find the PHY)
constexpr uint16_t PHY_PHYSCSR    = 0x001F;  // LAN8742 PHY Special Control/Status (negotiated speed)

constexpr uint16_t BCR_AUTONEG_EN      = 0x1000;  // bit 12: enable auto-negotiation
constexpr uint16_t BCR_RESTART_AUTONEG = 0x0200;  // bit 9:  restart auto-negotiation

constexpr uint16_t BSR_LINK_UP      = 0x0004;  // bit 2: link is up
constexpr uint16_t BSR_AUTONEG_DONE = 0x0020;  // bit 5: auto-negotiation complete

constexpr uint16_t PHYSCSR_SPEED_MASK = 0x001C;  // bits [4:2]: negotiated speed/duplex
constexpr uint16_t PHYSCSR_10BT_HD    = 0x0004;
constexpr uint16_t PHYSCSR_10BT_FD    = 0x0014;
constexpr uint16_t PHYSCSR_100BTX_HD  = 0x0008;
constexpr uint16_t PHYSCSR_100BTX_FD  = 0x0018;

constexpr uint32_t PHY_LINK_TIMEOUT_MS   = 3000;  // bounded so a missing link never hangs bring-up
constexpr uint32_t LINK_POLL_INTERVAL_MS = 1000;  // how often the loop re-checks the PHY link

// PHY link state carried between init() and the loop's link monitor. s_phy_addr is the address
// discovered on MDIO (-1 until found); s_link_up is the last observed link state, so the monitor
// only reconfigures the MAC on a down->up edge (e.g. a switch plugged in after boot).
int      s_phy_addr       = -1;
bool     s_link_up        = false;
uint32_t s_last_link_poll = 0;

// Scan the MDIO bus for the PHY: its strap address is board-dependent and was never used by this
// firmware, so probe 0..31 and take the first that answers with a sane ID. Returns -1 if none do.
int find_phy_address()
{
    for (uint32_t addr = 0; addr < 32; ++addr) {
        uint32_t id1 = 0;
        if (HAL_ETH_ReadPHYRegister(&heth, addr, PHY_PHYSID1, &id1) != HAL_OK) {
            continue;
        }
        if (id1 != 0x0000 && id1 != 0xFFFF) {
            return static_cast<int>(addr);
        }
    }
    return -1;
}

// Read the PHY's negotiated speed/duplex and push it into the MAC (without this the MAC keeps
// HAL_ETH_Init's fixed default). Shared by the init bring-up and the loop's link monitor.
bool apply_negotiated_speed(uint32_t addr)
{
    uint32_t scsr = 0;
    if (HAL_ETH_ReadPHYRegister(&heth, addr, PHY_PHYSCSR, &scsr) != HAL_OK) {
        return false;
    }
    ETH_MACConfigTypeDef mac = {};
    HAL_ETH_GetMACConfig(&heth, &mac);
    switch (scsr & PHYSCSR_SPEED_MASK) {
        case PHYSCSR_100BTX_FD: mac.Speed = ETH_SPEED_100M; mac.DuplexMode = ETH_FULLDUPLEX_MODE; break;
        case PHYSCSR_100BTX_HD: mac.Speed = ETH_SPEED_100M; mac.DuplexMode = ETH_HALFDUPLEX_MODE; break;
        case PHYSCSR_10BT_FD:   mac.Speed = ETH_SPEED_10M;  mac.DuplexMode = ETH_FULLDUPLEX_MODE; break;
        case PHYSCSR_10BT_HD:   mac.Speed = ETH_SPEED_10M;  mac.DuplexMode = ETH_HALFDUPLEX_MODE; break;
        default: return false;  // unexpected code — leave the MAC at its current config
    }
    HAL_ETH_SetMACConfig(&heth, &mac);
    return true;
}

// Force auto-negotiation on (overriding any strap that left the PHY forced/disabled), wait
// — bounded, so bring-up can never hang — for link + auto-neg complete, then push the negotiated
// speed/duplex into the MAC. Records the PHY address + link state for the loop monitor. Returns
// true if a link was negotiated and the MAC configured; false on no PHY / MDIO error / timeout
// (the main loop still runs and the link monitor picks the link up if it appears later).
bool configure_phy_link()
{
    s_phy_addr = find_phy_address();
    if (s_phy_addr < 0) {
        return false;  // no PHY responded on MDIO — wrong address strap or an electrical fault
    }
    const uint32_t addr = static_cast<uint32_t>(s_phy_addr);

    uint32_t bcr = 0;
    if (HAL_ETH_ReadPHYRegister(&heth, addr, PHY_BCR, &bcr) != HAL_OK) {
        return false;
    }
    bcr |= (BCR_AUTONEG_EN | BCR_RESTART_AUTONEG);
    if (HAL_ETH_WritePHYRegister(&heth, addr, PHY_BCR, bcr) != HAL_OK) {
        return false;
    }

    // Bounded wait for link-up AND auto-negotiation complete.
    constexpr uint16_t LINK_READY = BSR_LINK_UP | BSR_AUTONEG_DONE;
    const uint32_t start = HAL_GetTick();
    uint32_t bsr = 0;
    do {
        if (HAL_ETH_ReadPHYRegister(&heth, addr, PHY_BSR, &bsr) != HAL_OK) {
            return false;
        }
        if ((bsr & LINK_READY) == LINK_READY) {
            break;
        }
    } while ((HAL_GetTick() - start) < PHY_LINK_TIMEOUT_MS);

    if ((bsr & LINK_READY) != LINK_READY) {
        return false;  // no link within the window — the loop monitor adopts it once it appears
    }

    s_link_up = apply_negotiated_speed(addr);
    return s_link_up;
}

// Hot-plug link monitor for the main loop: non-blocking and rate-limited (one short MDIO read per
// LINK_POLL_INTERVAL_MS, a cheap tick comparison otherwise). On a down->up edge — a switch/cable
// plugged in after boot, or a switch that finished negotiating past the init window — it adopts
// the freshly negotiated speed/duplex. The PHY re-negotiates in hardware on link-up (auto-neg was
// enabled at init), so we only need to detect the edge and re-sync the MAC.
void poll_phy_link()
{
    const uint32_t now = HAL_GetTick();
    if ((now - s_last_link_poll) < LINK_POLL_INTERVAL_MS) {
        return;
    }
    s_last_link_poll = now;

    if (s_phy_addr < 0) {
        s_phy_addr = find_phy_address();   // PHY may not have answered at init; keep trying
        if (s_phy_addr < 0) {
            return;
        }
    }
    const uint32_t addr = static_cast<uint32_t>(s_phy_addr);

    uint32_t bsr = 0;
    if (HAL_ETH_ReadPHYRegister(&heth, addr, PHY_BSR, &bsr) != HAL_OK) {
        return;
    }
    constexpr uint16_t LINK_READY = BSR_LINK_UP | BSR_AUTONEG_DONE;
    const bool up_now = (bsr & LINK_READY) == LINK_READY;

    if (up_now && !s_link_up) {
        apply_negotiated_speed(addr);   // rising edge: the link just came up — adopt its speed
    }
    s_link_up = up_now;
}

} // namespace

/* -------------------------------------------------------------------------- */
/* Platform entry points                                                      */
/* -------------------------------------------------------------------------- */

namespace platform::communication::ethernet {

void Ethernet::init()
{
    // Mark the whole pool Free BEFORE starting the peripheral: HAL_ETH_Start_IT immediately
    // allocates ETH_RX_DESC_CNT buffers through HAL_ETH_RxAllocateCallback and marks them
    // OwnedDma. Initialising after Start would clobber those marks back to Free, so the pool
    // would later hand the DMA-owned buffers out a second time (double-use). Order matters.
    for (std::size_t i = 0; i < RX_BUF_COUNT; ++i) {
        rx_buf_status[i] = BufStatus::Free;
    }

    // Bring the PHY up and match the MAC to the negotiated link BEFORE starting the MAC/DMA.
    // MX_ETH_Init never touches the PHY, so the link otherwise relied on the reset straps (which
    // came up on a direct PC but not through a switch) and the MAC ran at a fixed default speed.
    // Bounded internally, so a missing link delays bring-up by at most PHY_LINK_TIMEOUT_MS rather
    // than hanging; the link can still come up later (the GS just sees no telemetry until it does).
    (void)configure_phy_link();

    HAL_ETH_Start_IT(&heth);

    std::memset(&TxConfig, 0, sizeof(TxConfig));
    TxConfig.Attributes   = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    TxConfig.CRCPadCtrl   = ETH_CRC_PAD_INSERT;
    TxConfig.TxBuffer     = &tx_eth_hdr_buf;

    tx_eth_hdr_buf.buffer = reinterpret_cast<uint8_t*>(&tx_eth_hdr);
    tx_eth_hdr_buf.len    = sizeof(tx_eth_hdr);
    tx_eth_hdr_buf.next   = nullptr;
    std::memcpy(tx_eth_hdr.src, heth.Init.MACAddr, 6);

    tx_ipv4_hdr_buf.buffer = reinterpret_cast<uint8_t*>(&tx_ipv4_hdr);
    tx_ipv4_hdr_buf.len    = sizeof(tx_ipv4_hdr);
    tx_ipv4_hdr_buf.next   = nullptr;
    tx_ipv4_hdr.version = 4;
    tx_ipv4_hdr.ihl     = 5;
    tx_ipv4_hdr.dscp    = 0;
    tx_ipv4_hdr.ecn     = 0;
    tx_ipv4_hdr.id      = 0;
    tx_ipv4_hdr.frag    = hton16(IPV4_DF_FLAG);
    tx_ipv4_hdr.ttl     = IPV4_DEFAULT_TTL;
    tx_ipv4_hdr.src     = hton32(LOCAL_IP);

    tx_icmp_reply_buf.buffer = tx_icmp_reply;
    tx_icmp_reply_buf.next   = nullptr;

    tx_udp_hdr_buf.buffer = reinterpret_cast<uint8_t*>(&tx_udp_hdr);
    tx_udp_hdr_buf.len    = sizeof(tx_udp_hdr);
    tx_udp_hdr_buf.next   = &tx_udp_data_buf;
    tx_udp_hdr.src_port   = hton16(LOCAL_PORT);

    tx_udp_data_buf.buffer = tx_udp_data;
    tx_udp_data_buf.next   = nullptr;

    tx_arp_buf.buffer = reinterpret_cast<uint8_t*>(&tx_arp);
    tx_arp_buf.len    = sizeof(ArpPacket);
    tx_arp_buf.next   = nullptr;
    tx_arp.htype = hton16(ARP_HTYPE_ETHERNET);
    tx_arp.ptype = hton16(ARP_PTYPE_IPV4);
    tx_arp.hlen  = ARP_HLEN_ETHERNET;
    tx_arp.plen  = ARP_PLEN_IPV4;
    std::memcpy(tx_arp.sha, heth.Init.MACAddr, 6);
    tx_arp.spa = hton32(LOCAL_IP);

    s_info.status.initialized = 1u;
    s_info.state              = EthernetState::Up;
}

/* ---- logic::communication::Ethernet contract ----------------------------- */

void Ethernet::tick()
{
    static std::size_t rx_read_idx = 0;

    poll_phy_link();   // hot-plug: adopt a link (and its negotiated speed) that comes up post-boot

    while (rx_queue_size) {
        if (rx_buf_status[rx_read_idx] != BufStatus::OwnedCpu) {
            break;  // buffer not ready yet
        }

        process_eth_frame(rx_pool[rx_read_idx]);

        rx_buf_status[rx_read_idx] = BufStatus::Free;
        rx_read_idx = (rx_read_idx + 1) % RX_BUF_COUNT;

        __disable_irq();
        rx_queue_size = static_cast<uint8_t>(rx_queue_size - 1);
        __enable_irq();
    }

    // RX self-recovery + overload protection. A burst that outpaces this loop — broadcast/multicast
    // chatter on the switch, especially while a slow loop iteration (an SD flush) keeps us out of
    // here — can drain the buffer pool dry. The RX DMA then un-arms its descriptors and SUSPENDS
    // (RBU); the RX interrupt stops firing, so the ISR drain never re-arms it and reception jams for
    // good. We freed buffers above, so re-run the drain here from the loop: HAL_ETH_ReadData re-arms
    // the descriptors and the DMA resumes (it picks up an owned descriptor again on the next frame —
    // guaranteed under live switch traffic). Guarded against the RX ISR (HAL_ETH_ReadData is not
    // reentrant). Net effect: excess traffic is DROPPED, never jammed, and RX is starved for at most
    // one loop iteration. In steady state this is one cheap HAL_ETH_ReadData call that finds nothing.
    HAL_NVIC_DisableIRQ(ETH_IRQn);
    __HAL_ETH_DMA_CLEAR_FLAG(&heth, ETH_DMA_RX_BUFFER_UNAVAILABLE_FLAG);
    uint8_t* recovered = nullptr;
    while (HAL_ETH_ReadData(&heth, reinterpret_cast<void**>(&recovered)) == HAL_OK) {
        const std::size_t idx =
            (reinterpret_cast<uintptr_t>(recovered) - reinterpret_cast<uintptr_t>(rx_pool)) / RX_BUF_SIZE_BYTES;
        if (idx < RX_BUF_COUNT) {
            rx_buf_status[idx] = BufStatus::OwnedCpu;
            rx_queue_size = static_cast<uint8_t>(rx_queue_size + 1);
        }
        recovered = nullptr;
    }
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}

std::optional<NetError> Ethernet::send(const Endpoint& dest, std::span<const uint8_t> payload)
{
    if (payload.size() > MAX_UDP_PAYLOAD_BYTES) {
        s_info.status.tx_error = 1u;
        return NetError::InternalError;
    }

    const auto len = static_cast<uint16_t>(payload.size());
    std::memcpy(tx_udp_data, payload.data(), len);

    prepare_eth_header(dest.mac.data(), ETHERTYPE_IPV4);
    prepare_ipv4_header(sizeof(UdpHeader) + len, IPV4_PROTOCOL_UDP, dest.ipv4);
    prepare_udp_header(len, dest.port);

    tx_udp_data_buf.len = len;
    TxConfig.Length = sizeof(EthHeader) + sizeof(Ipv4Header) + sizeof(UdpHeader) + len;

    switch (HAL_ETH_Transmit_IT(&heth, &TxConfig)) {
        case HAL_OK:
            s_info.status.tx_busy  = 0u;
            s_info.status.tx_error = 0u;
            return std::nullopt;
        case HAL_BUSY:
            s_info.status.tx_busy = 1u;
            return NetError::Busy;
        default:
            s_info.status.tx_error = 1u;
            return NetError::InternalError;
    }
}

std::optional<Datagram> Ethernet::receive()
{
    if (rx_ring_tail == rx_ring_head) {
        return std::nullopt;  // ring empty
    }

    const RxSlot& slot = rx_ring[rx_ring_tail];
    rx_ring_tail = (rx_ring_tail + 1) % RX_RING_CAPACITY_DATAGRAMS;

    Datagram datagram;
    datagram.source  = slot.source;
    datagram.payload = std::span<const uint8_t>(slot.data.data(), slot.length_bytes);
    return datagram;
}

EthernetInfo Ethernet::info() const
{
    return s_info;
}

} // namespace platform::communication::ethernet

/* -------------------------------------------------------------------------- */
/* HAL ETH RX DMA callbacks (producer side of the RX pool)                    */
/* -------------------------------------------------------------------------- */

extern "C" void HAL_ETH_RxAllocateCallback(uint8_t** buff)
{
    static std::size_t alloc_idx = 0;

    for (std::size_t i = 0; i < RX_BUF_COUNT; ++i) {
        if (rx_buf_status[alloc_idx] == BufStatus::Free) {
            rx_buf_status[alloc_idx] = BufStatus::OwnedDma;
            *buff = rx_pool[alloc_idx];
            alloc_idx = (alloc_idx + 1) % RX_BUF_COUNT;
            return;
        }
        alloc_idx = (alloc_idx + 1) % RX_BUF_COUNT;
    }
    *buff = nullptr;
}

extern "C" void HAL_ETH_RxLinkCallback(void** p_start, void** p_end, uint8_t* buff, uint16_t length)
{
    *p_start = buff;
    *p_end   = buff + length;
}

extern "C" void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef* eth_handle)
{
    // Drain EVERY completed frame, not just one. HAL_ETH_ReadData returns a single frame per
    // call (it stops at the first complete packet), and the ETH RX interrupt does NOT re-fire
    // for frames already waiting in the descriptor ring — so reading one frame per interrupt
    // strands the rest whenever several arrive between ISR entries (a debugger halt fills all
    // four descriptors; a burst can too). The stranded descriptors are then never re-armed
    // (ETH_UpdateDescriptor runs only inside ReadData), the DMA runs out of RX descriptors and
    // command reception stops dead while telemetry TX keeps running. Looping until ReadData
    // reports no more ready frames empties the ring and re-arms each descriptor from the buffer
    // pool (RX_BUF_COUNT > ETH_RX_DESC_CNT guarantees a spare buffer for the re-arm) — the same
    // drain loop ST's reference LwIP port uses.
    uint8_t* rx_buf = nullptr;
    while (HAL_ETH_ReadData(eth_handle, reinterpret_cast<void**>(&rx_buf)) == HAL_OK) {
        const std::size_t idx =
            (reinterpret_cast<uintptr_t>(rx_buf) - reinterpret_cast<uintptr_t>(rx_pool)) / RX_BUF_SIZE_BYTES;
        if (idx < RX_BUF_COUNT) {
            rx_buf_status[idx] = BufStatus::OwnedCpu;
            __disable_irq();
            rx_queue_size = static_cast<uint8_t>(rx_queue_size + 1);
            __enable_irq();
        }
        rx_buf = nullptr;
    }
}
