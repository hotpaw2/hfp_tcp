## hfp_tcp

This repository contains source code for hfp_tcp, 
an rtl_tcp "work-alike" server
for the AirSpy
[HF+](https://airspy.com/airspy-hf-plus/) 
and 
[HF+ Discovery](https://airspy.com/airspy-hf-discovery/)
SDR receivers.

Note that hfp_tcp is not the same thing as, 
nor compatible with, the spyserver protocol.
[rtl_tcp](https://github.com/osmocom/rtl-sdr)
is a command-line tool developed by OsmoCom and others,
for various Realtek RTL2832-based DVB-T USB peripherals,
to serve IQ samples from those USB devices over TCP.
There are multiple SDR applications,
available for Linux, macOS, or Wintel systems,
that can connect to a local or remote SDR radio peripheral
via the rtl_tcp protocol.

The hfp_tcp server allows many SDR applications 
that support the
HL2 sample rates (768k, 192k, etc.) to connect to
an AirSpy HF+ or HF+ Discovery.
Among the SDR applications 
supporting those sample rates
are the iOS and macOS apps:
[rtl_tcp SDR](http://www.hotpaw.com/rhn/hotpaw/)
and
SDR Receiver,
which allow using an hfp_tcp server from an iPhone, iPad, or Mac.

Dependancies for building:
[libusb](https://github.com/libusb/libusb)
and
[libairspyhf](https://github.com/airspy/airspyhf) . 
See instruction in those repositories for how to build them.  
Builds have been tested on Raspbian for the Raspberry Pi 3B and 4, 
and on macOS.

Usage:

    hfp_tcp -a server_IP_Address [-p tcp_server_port] [-b 8/16]

Starts a server for the rtl_tcp protocol
    on a local TCP server port (default rtl_tcp port 1234)
    and waits for a TCP connection.

Distribution License: BSD 3-clause
No warrantees implied.

--
