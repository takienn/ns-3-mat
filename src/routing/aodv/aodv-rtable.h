/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Based on 
 *      NS-2 AODV model developed by the CMU/MONARCH group and optimized and
 *      tuned by Samir Das and Mahesh Marina, University of Cincinnati;
 * 
 *      AODV-UU implementation by Erik Nordström of Uppsala University
 *      http://core.it.uu.se/core/index.php/AODV-UU
 *
 * Authors: Elena Borovkova <borovkovaes@iitp.ru>
 *          Pavel Boyko <boyko@iitp.ru>
 */
#ifndef __aodv_rtable_h__
#define __aodv_rtable_h__

#include <cassert>
#include <map>
#include <sys/types.h>
#include "ns3/ipv4.h"
#include "ns3/ipv4-route.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/timer.h"
#include "ns3/net-device.h"

namespace ns3 {
namespace aodv {

#define INFINITY2       0xff

/**
 * \ingroup aodv
 * \brief Route record states
 */
enum RouteFlags
{
  RTF_DOWN = 0,     //!< RTF_DOWN
  RTF_UP = 1,       //!< RTF_UP
  RTF_IN_REPAIR = 2,//!< RTF_IN_REPAIR
  RTF_IN_SEARCH = 3 //!< RTF_IN_SEARCH
};

/**
 * \ingroup aodv
 * \brief Routing table entry
 */
class RoutingTableEntry
{
public:
  RoutingTableEntry(Ptr<NetDevice> dev = 0,Ipv4Address dst = Ipv4Address(), bool vSeqNo = false, u_int32_t m_seqNo = 0,
      Ipv4InterfaceAddress iface = Ipv4InterfaceAddress(), u_int16_t  hops = 0,
      Ipv4Address nextHop = Ipv4Address(), Time lifetime = Simulator::Now());

  ~RoutingTableEntry();
  
  ///\name Precursors management
  //\{
  /**
   * Insert precursor in precursor list if it doesn't yet exist in the list
   * \param id precursor address
   * \return true on success
   */
  bool InsertPrecursor(Ipv4Address id);
  /**
   * Lookup precursor by address
   * \param id precursor address
   * \return true on success
   */
  bool LookupPrecursor(Ipv4Address id);
  /**
   * \brief Delete precursor
   * \param id precursor address
   * \return true on success
   */
  bool DeletePrecursor(Ipv4Address id);
  /// Delete all precursors
  void DeleteAllPrecursors();
  /**
   * \return true if precursor list empty
   */
  bool IsPrecursorListEmpty() const;
  /**
   * Inserts precursors in vector prec if they does not yet exist in vector
   */
  void GetPrecursors(std::vector<Ipv4Address> & prec) const;
  //\}

  /// Mark entry as "down" (i.e. disable it)
  void Invalidate (Time badLinkLifetime);
  ///\name Fields
  //\{
  Ipv4Address GetDestination() const { return m_ipv4Route->GetDestination(); }
  Ptr<Ipv4Route> GetRoute() const { return  m_ipv4Route; }
  void SetRoute(Ptr<Ipv4Route> r) { m_ipv4Route = r; }
  void SetNextHop (Ipv4Address nextHop) { m_ipv4Route->SetGateway(nextHop); }
  Ipv4Address GetNextHop () const { return m_ipv4Route->GetGateway(); }
  void SetOutputDevice(Ptr<NetDevice> dev) { m_ipv4Route->SetOutputDevice(dev); }
  Ptr<NetDevice> GetOutputDevice() const { return m_ipv4Route->GetOutputDevice(); }
  Ipv4InterfaceAddress GetInterface() const { return m_iface;}
  void SetInterface (Ipv4InterfaceAddress iface) { m_iface = iface; }
  void SetValidSeqNo(bool s) { m_validSeqNo = s; }
  bool GetValidSeqNo() const { return m_validSeqNo; }
  void SetSeqNo(uint32_t sn) { m_seqNo = sn; }
  uint32_t GetSeqNo() const { return m_seqNo; }
  void SetHop(uint16_t hop) { m_hops = hop; }
  uint16_t GetHop() const {return m_hops; }
  void SetLifeTime(Time lt) { m_lifeTime = lt + Simulator::Now(); }
  Time GetLifeTime() const { return m_lifeTime - Simulator::Now(); }
  void SetFlag(uint8_t flag) { m_flag = flag; }
  uint8_t GetFlag() const { return m_flag; }
  void SetRreqCnt(uint8_t n) { m_reqCount = n; }
  uint8_t GetRreqCnt() const { return m_reqCount; }
  void IncrementRreqCnt() { m_reqCount++; }
  bool IsInBlacklist() const { return m_blackListState; }
  void SetUnidirectional(bool u) { m_blackListState = u; }
  void SetBalcklistTimeout (Time t) { m_blackListTimeout = t; }
  Time GetBlacklistTimeout () { return m_blackListTimeout; }
  //\}

  /**
   * \brief Compare destination address
   * \return true if equal
   */
  bool operator==(Ipv4Address const  dst) const
  {
    return (m_ipv4Route->GetDestination() == dst);
  }
  void Print(std::ostream & os) const;

private:
  /// Valid Destination Sequence Number flag
  bool m_validSeqNo;
  /// Destination Sequence Number, if m_validSeqNo = true
  uint32_t m_seqNo;
  /// Hop Count (number of hops needed to reach destination)
  uint16_t m_hops;
  /**
  * \brief Expiration or deletion time of the route
  *	Lifetime field in the routing table plays dual role --
  *	for an active route it is the expiration time, and for an invalid route
  *	it is the deletion time.
  */
  Time m_lifeTime;
  /** Ip route, include
  *   - destination address
  *   - source address
  *   - next hop address (gateway)
  *   - output device
  */
  Ptr<Ipv4Route> m_ipv4Route;
  /// Output interface address
  Ipv4InterfaceAddress m_iface;
  /// Routing flags: down, up or in repair
  uint8_t m_flag;

  /// List of precursors
  std::vector<Ipv4Address> m_precursorList;
  /// When I can send another request
  Time m_routeRequestTimout;
  /// Number of route requests
  uint8_t m_reqCount;
  /// Indicate if this entry is in "blacklist"
  bool m_blackListState;
  Time m_blackListTimeout;
public:
  /// RREP_ACK timer
  Timer m_ackTimer;
  /// LifeTime timer of this entry
  Timer lifeTimeTimer;
};

/**
 * \ingroup aodv
 * The Routing table used by AODV protocol
 */
class RoutingTable
{
public:
  RoutingTable() {}

  /**
   * Add routing table entry if it doesn't yet exist in routing table
   * \param r routing table entry
   * \return true in success
   */
  bool AddRoute(RoutingTableEntry & r);
  /**
   * Delete routing table entry
   * \param dst destination address
   * \return true on success
   */
  bool DeleteRoute(Ipv4Address dst);
  /**
   * Lookup routing table entry
   * \param dst destination address
   * \param rt entry with destination address dst, if exists
   * \return true on success
   */
  bool LookupRoute(Ipv4Address dst, RoutingTableEntry & rt) const;
  /// Update routing table
  bool Update(Ipv4Address dst, RoutingTableEntry & rt);
  /// Set routing table entry flags
  void SetEntryState (Ipv4Address dst, uint8_t state /*TODO use enum*/);
  /**
   * Lookup valid routing entries with next hop Address dst and not empty list of precursors.
   */
  void GetListOfDestinationWithNextHop(Ipv4Address nextHop, std::map<Ipv4Address, uint32_t> unreachable);
  /**
   *   Update routing entries with this destinations as follows:
   *  1. The destination sequence number of this routing entry, if it
   *     exists and is valid, is incremented.
   *  2. The entry is invalidated by marking the route entry as invalid
   *  3. The Lifetime field is updated to current time plus DELETE_PERIOD.
   */
  void InvalidateRoutesWithDst(std::map<Ipv4Address, uint32_t> const & unreachable, Time badLinkLifetime);
  /// Delete all outdated entries and invalidate valid entry if Lifetime is expired
  void Purge(Time badLinkLifetime);
  /** Mark entry as unidirectional (e.g. add this neighbor to "blacklist" for blacklistTimeout period)
   * \param neighbor - neighbor address link to which assumed to be unidirectional
   * \return true on success
   */
  bool MarkLinkAsUinidirectional(Ipv4Address neighbor, Time blacklistTimeout);
  /// Print routing table
  void Print(std::ostream &os) const;

private:
  std::map<Ipv4Address, RoutingTableEntry> m_ipv4AddressEntry;
};

}}

#endif /* _aodv__rtable_h__ */
