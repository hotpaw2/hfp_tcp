//
//  hfp_tcp_server.c
//
//  Serves IQ data using the rtl_tcp protocol
//    from an Airspy HF+
//    on iPv6 port 1234
//
#define VERSION "v.1.2.116" // 116b4
//   v.1.2.116 2020-08-25  rhn 
//   v.1.2.112 2019-07-30  0am barry@medoff.com
//   v.1.2.111 2019-05-05  2pm  rhn
//   v.1.2.109 2019.05.04 10pm barry@medoff.com
//   Copyright 2017,2019 Ronald H Nicholson Jr. All Rights Reserved.
//   re-distribution under the BSD 3 clause license permitted
//
//   pi :    
//   	cc -std=c99 -lm -lairspyhf -lpthread -Os -o hfp_tcp hfp_tcp_server.c
//
//   macOS : 
//	clang -lm -llibairspyhf -lpthread -Os -o hfp_tcp hfp_tcp_server.c 
//   					// libairspyhf.1.6.8.dylib
//
//   requires these 2 files to compile
//     airspyhf.h
//     libairspyhf.1.6.8.dylib or /usr/local/lib/libairspyhf.so.1.6.8
//   from libairspyhf at https://github.com/airspy/airspyhf
//

#define SOCKET_READ_TIMEOUT_SEC ( 10.0 * 60.0 )
#define SAMPLE_BITS     ( 8)    // default to match rtl_tcp
// #define SAMPLE_BITS  (16)    // default to match rsp_tcp
// #define SAMPLE_BITS  (32)    // HF+ capable of float32 IQ data
#define GAIN8           (64.0)  // default gain
#define PORT            (1234)  // default port
#define RING_BUFFER_ALLOCATION  (2L * 8L * 1024L * 1024L)  // 16MB

#define _POSIX_C_SOURCE 200112L
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <inttypes.h>
#include <unistd.h>
#include <math.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <pthread.h>
#include <sys/time.h>

#ifndef _UNISTD_H_
int usleep(unsigned long int usec);
#endif

#include <sys/time.h>

#include "airspyhf.h"

void *connection_handler(void);
void *tcp_send_handler(void *param);
int usb_rcv_callback(airspyhf_transfer_t *context);
static void sighandler(int signum);

uint64_t            serialnum   =  0;
airspyhf_device_t   *device     =  NULL;
airspyhf_transfer_t context;

int             sendErrorFlag   =  0;
int             sampleBits      =  SAMPLE_BITS;
int         numSampleRates      =  1;
static long int totalSamples    =  0;
long        sampRate            =  768000;
long        previousSRate       = -1;
float       gain0               =  GAIN8;
int        gClientSocketID      = -1;

uint8_t	   *ring_buffer_ptr     =  NULL;
int		decimateFlag	=  1;
int		decimateCntr	=  0;
int		filterFlag	=  0;
void 	iir_fbc(float *s, int n, int order);
void 	init_iir();

static int    listen_sockfd;
struct sigaction    sigact, sigign;
static volatile int     do_exit =  0;
float        acc_r              =  0.0;    // accumulated rounding
float        sMax               =  0.0;    // for debug
float        sMin               =  0.0;
int		sendblockcount  =  0;
int 		threads_running =  0;

char UsageString[]
    = "Usage:    [-p listen port (default: 1234)]\n          [-b 16]";

int main(int argc, char *argv[]) {

    struct sockaddr_in6 serv_addr ;
    char client_addr_ipv6[100];
    int portno     =  PORT;     //
    char *ipaddr =  NULL;       // "127.0.0.1"
    int n;

    if (argc > 1) {
    if ((argc % 2) != 1) {
            printf("%s\n", UsageString);
            exit(0);
        }
        for (int arg=3; arg<=argc; arg+=2) {
        if (strcmp(argv[arg-2], "-p")==0) {
                portno = atoi(argv[arg-1]);
                if (portno == 0) {
                    printf("invalid port number entry %s\n", argv[arg-1]);
                    exit(0);
                }
            } else if (strcmp(argv[arg-2], "-b")==0) {
                if (strcmp(argv[arg-1],"16")==0) {
                    sampleBits = 16;
                } else if (strcmp(argv[arg-1],"8")==0) {
                    sampleBits =  8;
                } else {
                    printf("%s\n", UsageString);
                    exit(0);
                }
            } else if (strcmp(argv[arg-2], "-a")==0) {
        ipaddr = argv[arg-1];        // unused
            } else {
                printf("%s\n", UsageString);
                exit(0);
            }
        }
    }

    printf("\nhfp_tcp Version %s\n\n", VERSION);

    ring_buffer_ptr = (uint8_t *)malloc(RING_BUFFER_ALLOCATION + 4);
    if (ring_buffer_ptr == NULL) { exit(-1); }
    bzero(ring_buffer_ptr, RING_BUFFER_ALLOCATION + 2);

    printf("Serving %d-bit samples on port %d\n", sampleBits, portno);

    uint64_t serials[4] = { 0L,0L,0L,0L };
    int count = 2;
    n = airspyhf_list_devices(&serials[0], count);
    printf("hf+ devices = %d\n", n);
    if (n == 0L) { exit(-1); }
    printf("hf+ serial# = ");
    uint64_t t = serials[0];
    printf("%" PRIu64 "\n", t);
    serialnum = t;
    if (serialnum == 0L) { exit(-1); }

    sigact.sa_handler = sighandler;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigaction(SIGINT,  &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);
    sigaction(SIGQUIT, &sigact, NULL);
#ifdef __APPLE__
    signal(SIGPIPE, SIG_IGN);
#else
    sigign.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sigign, NULL);
#endif

    n = airspyhf_open_sn(&device, serialnum);
    printf("hf+ open status = %d\n", n);
    if ((n < 0) || (device == NULL)) { exit(-1); }

    airspyhf_lib_version_t version;
    airspyhf_lib_version(&version);
    printf("\nlibairspyhf   %" PRIu32 ".%" PRIu32 ".%" PRIu32 "\n",
           version.major_version, version.minor_version, version.revision);

    char versionString[64];
    uint8_t versionLength = 64;

    bzero((char *)&versionString[0], 64);

    n = airspyhf_version_string_read(device, &versionString[0], versionLength);
    if (n == AIRSPYHF_ERROR) {
    printf("Error reading version string");
    exit(-1);
    }
    printf("hf+ firmware %s\n\n", versionString);

    uint32_t sr_buffer[100];
    airspyhf_get_samplerates(device, sr_buffer, 0);
    uint32_t sr_len = sr_buffer[0];
    printf("number of supported sample rates: %d \n", sr_len);
    if (sr_len > 0 && sr_len < 100) {
      numSampleRates     =  sr_len;
      airspyhf_get_samplerates(device, sr_buffer, sr_len);
      printf("supported sample rates: ");
        for (int i=0; i<sr_len; i++) {
          printf("%d ", sr_buffer[i]);
        }
        printf(" \n\n");
    }

    int sampRate = 768000;
    n = airspyhf_set_samplerate(device, sampRate);
    printf("set rate status = %d %d\n", sampRate, n);
    previousSRate = sampRate;
    long int f0 = 162450000;
    n = airspyhf_set_freq(device, f0);
    printf("set f0 status = %ld %d\n", f0, n);

    printf("\nhfp_tcp IPv6 server started on port %d\n", portno);

    listen_sockfd = socket(AF_INET6, SOCK_STREAM, 0);
    if (listen_sockfd < 0) {
        printf("ERROR opening socket");
        return(-1);
    }

    struct linger ling = {1,0};
    int rr = 1;
    setsockopt(listen_sockfd, SOL_SOCKET, SO_REUSEADDR,
            (char *)&rr, sizeof(int));
    setsockopt(listen_sockfd, SOL_SOCKET, SO_LINGER,
            (char *)&ling, sizeof(ling));

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin6_flowinfo = 0;
    serv_addr.sin6_family = AF_INET6;
    serv_addr.sin6_addr = in6addr_any;
    serv_addr.sin6_port = htons(portno);

    // Sockets Layer Call: bind()
    if (bind( listen_sockfd, (struct sockaddr *)&serv_addr,
             sizeof(serv_addr) ) < 0) {
        printf("ERROR on bind to listen\n");
        return(-1);
    }

    listen(listen_sockfd, 5);
    fprintf(stdout, "listening for socket connection \n");

    while (1) {

        // accept a connection

        struct sockaddr_in6 cli_addr;
        socklen_t claddrlen = sizeof(cli_addr);
        gClientSocketID = accept( listen_sockfd,
                                 (struct sockaddr *) &cli_addr,
                                 &claddrlen );
        if (gClientSocketID < 0) {
            printf("ERROR on accept\n");
            break;
        }

        inet_ntop(AF_INET6, &(cli_addr.sin6_addr), client_addr_ipv6, 100);
        printf("\nConnected to client with IP address: %s\n",
               client_addr_ipv6);

        connection_handler();

    }

    n = airspyhf_close(device);
    printf("hf+ close status = %d\n", n);

    fflush(stdout);
    return 0;
}  //  main

static void sighandler(int signum)
{
        fprintf(stderr, "Signal caught, exiting!\n");
        fflush(stderr);
        close(listen_sockfd);
        if (gClientSocketID != 0) {
            close(gClientSocketID);
            gClientSocketID = -1;
        }
        if (device != NULL) {
            airspyhf_close(device);
            device = NULL;
        }
    exit(-1);
        do_exit = 1;
}

int stop_send_thread = 0;
int thread_counter = 0;
int thread_running = 0;

int  ring_buffer_size   =  RING_BUFFER_ALLOCATION;
volatile long int ring_wr_index  =  0;
volatile long int ring_rd_index  =  0;

int ring_data_available() 
{
    long int n = 0;
    int w_index = ring_wr_index;  // 
    int r_index = ring_rd_index;  //
    n = w_index - r_index;
    if (n < 0) { n += ring_buffer_size; }
    if (n < 0) { n = 0; }	                // error condition ?
    if (n >= ring_buffer_size) { n = 0; }       // error condition
    return(n);
}

int ring_write(uint8_t *from_ptr, int amount)
{
    int wrap = 0;
    int w_index = ring_wr_index;  // my index
    int r_index = ring_rd_index;  // other threads index
    if (   (w_index < 0) 
        || (w_index >= ring_buffer_size) ) { return(-1); }  // error !
    if (decimateFlag > 1) {
        int i;
        if (sampleBits == 16) { 
          for (i = 0; i < amount; i += 4) {
            if (decimateCntr == 0) {
	      ring_buffer_ptr[w_index  ] = from_ptr[i  ];
	      ring_buffer_ptr[w_index+1] = from_ptr[i+1];
	      ring_buffer_ptr[w_index+2] = from_ptr[i+2];
	      ring_buffer_ptr[w_index+3] = from_ptr[i+3];
              w_index += 4;
              if (w_index >= ring_buffer_size) { w_index = 0; }
	    }
	    decimateCntr += 1;
	    if (decimateCntr >= decimateFlag) { decimateCntr = 0; }
	  }
	} else {	// assume samplebits == 8
          for (i = 0; i < amount; i += 2) {
            if (decimateCntr == 0) {
	      ring_buffer_ptr[w_index  ] = from_ptr[i  ];
	      ring_buffer_ptr[w_index+1] = from_ptr[i+1];
              w_index += 2;
              if (w_index >= ring_buffer_size) { w_index = 0; }
	    }
	    decimateCntr += 1;
	    if (decimateCntr >= decimateFlag) { decimateCntr = 0; }
	  }
	}
    } else if (w_index + amount < ring_buffer_size) {
        memcpy(&ring_buffer_ptr[w_index], from_ptr, amount);
        w_index += amount;
    } else {
        int i;
        for (i = 0; i < amount; i += 1) {
            ring_buffer_ptr[w_index] = from_ptr[i];
            w_index += 1;
            if (w_index >= ring_buffer_size) { w_index = 0; }
        }
    }
    // 
    // insert memory barrier here
    //
    ring_wr_index = w_index;	 // update lock free input info
// fprintf(stdout, "into ring %d\n", amount); // yyy yyy
// fflush(stdout);
    int m = ring_data_available();
    if (m > ring_buffer_size/2) { wrap = 1; }
    return(wrap);
}
        
int ring_read(uint8_t *to_ptr, int amount, int always)
{
    int bytes_read = 0;
    int r_index = ring_rd_index;  // my index
    int w_index = ring_wr_index;  // other threads index
    int available = w_index - r_index;
    if (available < 0) { available += ring_buffer_size; }
    if (always != 0) {
        bzero(to_ptr, amount);
    }
    if (available <= 0) { return(bytes_read); }
    int n = amount;
    if (n > available) { n = available; }	// min(n, available)
    if (r_index + n < ring_buffer_size) {
        memcpy(to_ptr, &ring_buffer_ptr[r_index], n);
        r_index += n;
    } else {
      int i;
      for (i = 0; i < n; i += 1) {
          to_ptr[i] = ring_buffer_ptr[r_index];
          r_index += 1;
          if (r_index >= ring_buffer_size) { r_index = 0; }
      }
    }
    bytes_read = n;
// fprintf(stderr, "out of ring %d\n", n); // yyy yyy
// fflush(stderr);
    ring_rd_index = r_index;  	 // update lock free extract info
    return(bytes_read);
}

float tmpFPBuf[4*32768];
uint8_t tmpBuf[4*32768];

void send_delay(int n, int rate)
{
    int n1 = n;					// n in bytes
    if (sampleBits == 16) { n1 = n / 2; }	// convert to samples
    if (sampleBits == 32) { n1 = n / 4; }	// convert to samples
    double dt = 0.10 * (double)n1 / rate; // fraction of a second
    unsigned int dt_uS = floor(1.0e6 * dt);	// 10X too fast
    usleep(dt_uS);
}

void *tcp_send_handler(void *param)
{
    int sz0   =     1408;                      // MTU size ? 
    int pad   =    32768 * 2;
    printf("send thread %d running 2 \n", thread_counter);
    while (stop_send_thread == 0) {
	if (gClientSocketID  <  0) { break; }
        if (ring_data_available() >= (sz0 + pad)) {
            int sz = ring_read(tmpBuf, sz0, 0);
	    if (sz > 0) {
                int k = 0;
		int send_sockfd = gClientSocketID ;
#ifdef __APPLE__
                k = send(send_sockfd, tmpBuf, sz, 0);
#else
                k = send(send_sockfd, tmpBuf, sz, MSG_NOSIGNAL);
#endif
                if (k <= 0) { sendErrorFlag = -1; }
// fprintf(stderr, "sent %d\n", k); // yyy yyy
// fflush(stderr);
                totalSamples   +=  sz;
                sendblockcount +=  1;
	    }
	    pad = 0;
	} else {
// fprintf(stdout, "send blocked \n" ); // yyy yyy
// fflush(stdout);
	        // send_delay(sz0, sampRate);
		usleep(1);
	}
    }
    pthread_exit(NULL);
    fprintf(stderr, "tcp send thread %d stopped\n", thread_counter);
    fflush(stderr);
    threads_running -= 1;
    stop_send_thread = 0;
    return(NULL);
}

void *connection_handler()
{
    char buffer[256];
    int n = 0;
    int m = 0;

    if (do_exit != 0) { return(NULL); }

    m = airspyhf_is_streaming(device);
    if (m > 0) {    // stop before restarting
        printf("hf+ already running = %d\n", m);
        fprintf(stdout,"stopping now 00 \n");
        m = airspyhf_stop(device);
        printf("hf+ stop status = %d\n", m);
        usleep(250L * 1000L);
    }

    if (1) {        // 16 or 12-byte rtl_tcp header
        int sz = 16;
        if (sampleBits == 8) { sz = 12; }
        // HFP0 16
        char header[16] = { 0x48,0x46,0x50,0x30, 
	    0x30,0x30,0x30+numSampleRates,0x30+sampleBits,
            0,0,0,1, 0,0,0,2 };
#ifdef __APPLE__
        n = send(gClientSocketID, header, sz, 0);
#else
        n = send(gClientSocketID, header, sz, MSG_NOSIGNAL);
#endif
        fprintf(stdout, "header sent %d\n", n); // yyy yyy
        fflush(stdout);
    }

    sendErrorFlag       =  0;
    stop_send_thread    =  0;
    ring_wr_index       =  0;
    ring_rd_index       =  0;
    pthread_t tcp_send_thread;
    long int *param = (long int *)malloc(4 * sizeof(long int));
    param[0]            =  0;
    if ( pthread_create( &tcp_send_thread, NULL ,
                             tcp_send_handler,
                             (void *)param) < 0) {
            printf("could not create tcp send thread");
            return(NULL);
    } else {
            printf("send thread started 1 \n");
    }

    acc_r         =  0.0;
    totalSamples  =  0;
    m = airspyhf_start(device, &usb_rcv_callback, &context);
    printf("hf+ start status = %d\n", m);
    if (m < 0) { exit(-1); }
    usleep(250L * 1000L);

    // set a timeout so receive call won't block forever
    struct timeval timeout;
    timeout.tv_sec = SOCKET_READ_TIMEOUT_SEC;
    timeout.tv_usec = 0;
    setsockopt( gClientSocketID, SOL_SOCKET, SO_RCVTIMEO,
               &timeout, sizeof(timeout) );

    n = 1;
    while ((n > 0) && (sendErrorFlag == 0)) {
        int i, j, m;
        // receive 5 byte commands (or a multiple thereof)
        memset(buffer,0, 256);
        n = recv(gClientSocketID, buffer, 255, 0);
        if ((n <= 0) || (sendErrorFlag != 0)) {
            if (airspyhf_is_streaming(device)) {
                fprintf(stdout,"stopping now 00 \n");
                m = airspyhf_stop(device);
            }
            close(gClientSocketID);
            gClientSocketID = -1;
            fprintf(stdout, "hf+ stop status = %d\n", m);
            fflush(stdout);
            break;
        }
        if (n > 0) {
            int msg1 = buffer[0];
            if (msg1 != 4) {
                for (i=0; i < n; i++) {
                    fprintf(stdout, "%02x ", (0x00ff & buffer[i]));
                }
                if (n > 0) { fprintf(stdout, "\n"); }
            }
            for (i=0; i < n; i+=5) {
                // decode 5 byte rtl_tcp command messages
                int msg  = buffer[i];
                int data = 0;
                for (j=1;j<5;j++) {
                    data = 256 * data + (0x00ff & buffer[i+j]);
                }

                if (msg == 1) {    // set frequency
                    int f0 = data;
                    fprintf(stdout, "setting frequency to: %d\n", f0);
                    m = airspyhf_set_freq(device, f0);
                    printf("set frequency status = %d\n", m);
                }
                if (msg == 2) {    // set sample rate
                    int r = data;
                    if ((r != previousSRate) || (decimateFlag > 1)) {
		        int restartflag = 0;
			if ((r == 48000) && (numSampleRates <= 4)) {
                          fprintf(stdout, 
			    "decimating 192k sample rate to 48k\n");
			  decimateFlag = 4;
			  init_iir();
			  filterFlag   = 1;
			  r = 4 * 48000; 	// 192000
			} else {
			  decimateFlag = 1;
			  filterFlag   = 0;
                          fprintf(stdout, "setting samplerate to: %d\n", r);
			}
                        sampRate = r;
    			m = airspyhf_is_streaming(device);
    			if (m > 0) {    // stop before restarting
                            fprintf(stdout,"stopping now 00 \n");
        		    m = airspyhf_stop(device);
		            restartflag = 1;
			    usleep(50L * 1000L);
			}
                        m = airspyhf_set_samplerate(device, sampRate);
                        printf("set samplerate status = %d\n", m);
                        previousSRate = r;
		        if (restartflag == 1) {
			    usleep(50L * 1000L);
                            m = airspyhf_start(device, 
			    		&usb_rcv_callback, &context);
                            fprintf(stdout, "hf+ start status = %d\n", m);
                            m = airspyhf_is_streaming(device);
                            fprintf(stdout, "hf+ is running = %d\n", m);
                            fflush(stdout);
			}
                    }
                }
                if (msg == 3) {            // other
                    fprintf(stdout, "message = %d, data = %d\n", msg, data);
                }
                if (msg == 4) {            // gain
                    if (   (sampleBits ==  8)
                        || (sampleBits == 16) ) {
                        // set gain ?
                        float g1 = data; // data : in 10th dB's
                        float g2 = 0.1 * (float)(data); // undo 10ths
                        fprintf(stdout, "setting gain to: %f dB\n", g2);
                        float g4 = g2 - 12.0; // ad hoc offset
                        float g5 = pow(10.0, 0.1 * g4); // convert from dB
                        gain0 = GAIN8 * g5;        // 64.0 = nominal
                        msg1 = msg;
                        float  g8  =  gain0; // GAIN8;
                        fprintf(stdout, "8b  gain multiplier = %f\n", g8);
                        float  g16 =   64.0 * gain0; // GAIN16;
                        fprintf(stdout, "16b gain multiplier = %f\n", g16);
            }
                }
                if (msg > 4) {            // other
                    fprintf(stdout, "message = %d, data = %d\n", msg, data);
                }
            }
            if (msg1 != 4) {
                m = airspyhf_is_streaming(device);
                printf("hf+ is running = %d\n", m);
                if (m == 0) {    // restart if command stops things
                    sendErrorFlag =  0;
                    m = airspyhf_start(device, &usb_rcv_callback, &context);
                    fprintf(stdout, "hf+ start status = %d\n", m);
                    m = airspyhf_is_streaming(device);
                    fprintf(stdout, "hf+ is running = %d\n", m);
                    fflush(stdout);
                }
            }
        }
        if (n < 0) {
            fprintf(stdout, "read socket timeout %d \n", n);
            fflush(stdout);
        }
        // loop until error (socket close) or timeout
    } ;

    m = airspyhf_is_streaming(device);
    printf("hf+ is running = %d\n", m);
    if (m) {
	fprintf(stdout,"stopping now 00 \n");
        m = airspyhf_stop(device);
        printf("hf+ stop status = %d\n", m);
    }

    close(gClientSocketID);
    gClientSocketID = -1;
    return(param);
} // connection_handler()

// uint8_t tmpBuf[4*32768];

typedef union
{
    uint32_t i;
    float    f;
} Float32_t;

float rand_float_co()
{
    Float32_t x;
    x.i = 0x3f800000 | (rand() & 0x007fffff);
    return(x.f - 1.0f);
}

int usb_rcv_callback(airspyhf_transfer_t *context)
{
    float  *p =  (float *)(context->samples);
    int    n  =  context->sample_count;
    int       sz ;

    if (sendErrorFlag != 0) { return(-1); }
    if (do_exit != 0) { return(-1); }
    //
    if ((sendblockcount % 1000) == 0) {
        fprintf(stdout,"+"); fflush(stdout);
    }
    //
    if (p != NULL && n > 0) {
        // fwrite(p, 8, n, file);
        uint8_t *dataBuffer    =  (uint8_t *)p; 	// unneeded line
	memcpy(&tmpFPBuf[0], p, 8*n);
	if (filterFlag != 0) {
            int order =  12;
	    iir_fbc(&tmpFPBuf[0], 2*n, order);
	}
	p = &tmpFPBuf[0];
        int    k        =  0;
        if (sampleBits ==  8) {
            float  g8  =  gain0; // GAIN8;
            // gain is typically 64.0
            // should be 128.0 or 2X larger, so 1-bit missing
            float rnd0A = rand_float_co();
            float rnd0B = rand_float_co();
            for (int i=0; i<2*n; i++) {
                float x;
                Float32_t x1;          // for debug hex print
                x    = p[i];
                float y = g8 * x;
                // add triangular noise
                // for noise filtered rounding
                float rnd1 = rand_float_co(); // noise with pdf [0..1)
                float r = rnd1 - (((i&1)==1) ? rnd0A : rnd0B);
                y = y + r;
                float ry = roundf(y);
                acc_r += (y - ry);     // for future noise filtering
                k = (int)ry;
                tmpBuf[i] = k + 128;
                if ((i&1) == 1) {      // round I
                    rnd0A = rnd1;      // save for next iteration
                } else {               // round Q
                    rnd0B = rnd1;      // save for next iteration
                }
            }
            // previous rounding
            /*
            for (int i=0; i<2*n; i++) {
                float x = g8 * p[i];
                int   k = (int)roundf(x);
                tmpBuf[i] = k + 128;  // 8-bit unsigned DC offset
            }
            */
            dataBuffer = (uint8_t *)(&tmpBuf[0]);
            sz = 2 * n;
        } else if (sampleBits == 16) {
            int16_t *tmp16ptr = (int16_t *)&tmpBuf[0];
            float  g16  =   64.0 * gain0; // GAIN16;
            // gain is typically 64.0 * 64.0 = 4096.0
            // should be 32768.0 or 8X larger, so 3-bits missing
            for (int i=0; i<2*n; i++) {
                float x = g16 * p[i];
                int   k = (int)roundf(x);
                tmp16ptr[i] = k;
            }
            dataBuffer = (uint8_t *)(&tmpBuf[0]);
            sz = 4 * n;
        } else {
            sz = 8 * n;    // two 32-bit floats for IQ == 8 bytes
        }
        int wrap = ring_write(dataBuffer, sz);
        if (wrap != 0) { 
            // fprintf(stderr, "ring wrap around error %d\n", wrap); // yyy
            // fflush(stderr);
	}
	wrap = 0; 				// yyy yyy
        if ((do_exit != 0) || (wrap != 0)) { 
            stop_send_thread = 0;
	    return(-1); 
	}
        totalSamples += n;
    }
    sendblockcount += 1;
    return(0);
}


//
//

typedef struct iirParams {
    float 	a0;
    float 	a1;
    float 	a2;
    float 	b0;	
    float 	b1;	
    float 	b2;	
    float 	ys_2L;	// saved history
    float 	ys_1L;
    float 	ys_0L;	// last output
    float 	xs_2L;
    float 	xs_1L;
    float 	xs_0L;	// last input
    float 	ys_2R;	// saved history
    float 	ys_1R;
    float 	ys_0R;	// last output
    float 	xs_2R;
    float 	xs_1R;
    float 	xs_0R;	// last input
    float	sr;
    float	cf;
    float	q;
    int32_t 	ftype;
} iirParams;

void iir_f2(float *s, int n, iirParams *p) // IQ or stereo
{
    float	a0, a1, a2, b0, b1, b2;
    float 	x2L,x1L,x0L,y2L,y1L,y0L;
    float 	x2R,x1R,x0R,y2R,y1R,y0R;
    int   	i, k;

    a0 = p->a0 ;
    a1 = p->a1 ;
    a2 = p->a2 ;
    b0 = p->b0 ;
    b1 = p->b1 ;
    b2 = p->b2 ;
    k  = p->ftype;

    x1L = p->xs_1L;			// recover history
    x0L = p->xs_0L;
    y1L = p->ys_1L;
    y0L = p->ys_0L;
    x1R = p->xs_1R;			// recover history
    x0R = p->xs_0R;
    y1R = p->ys_1R;
    y0R = p->ys_0R;
    if (k == 1) {			/* type 1 = lowpass  */
      for (i=0; i<n; i+=2) { 		// +=2 for interleaved
        x2L = s[i  ]; 
	y2L = b0 * x2L + b1 * x1L + b2 * x0L - a1 * y1L - a2 * y0L;
        s[i  ] = y2L;
        y0L = y1L; y1L = y2L;
        x0L = x1L; x1L = x2L;
	//
        x2R = s[i+1]; 
	y2R = b0 * x2R + b1 * x1R + b2 * x0R - a1 * y1R - a2 * y0R;
        s[i+1] = y2R;
        y0R = y1R; y1R = y2R;
        x0R = x1R; x1R = x2R;
      }
    }
    p->xs_1L = x1L;		// save history
    p->xs_0L = x0L;
    p->ys_1L = y1L;
    p->ys_0L = y0L;
    p->xs_1R = x1R;		// save history
    p->xs_0R = x0R;
    p->ys_1R = y1R;
    p->ys_0R = y0R;
}

void calc_iir_coefs(int ftype, float cf, float q, float sr, iirParams *p)
{
    double        w0, alpha;
    double        b0,b1,b2,a0,a1,a2;
    double	  g1, dbg;		// dB gain
    double        y;

    a0 = 0.0;
    a1 = 0.0;
    a2 = 0.0;
    b0 = 0.0;
    b1 = 0.0;
    b2 = 0.0;
    if (ftype == 3) {              // bandpass w/ 0 gain
	dbg = 0.0;
	g1 = sqrt(pow(10.0, (dbg / 20.0)));
        w0 = 2.0 * 3.14159265358979 * cf / sr;
	alpha = sin(w0)/(2.0 * q);
	b0 =  alpha;
	b1 =  0.0;
	b2 = -alpha;
	a0 =  1.0 + alpha;
	a1 = -2.0 * cos(w0);
	a2 =  1.0 - alpha;
    } else if (ftype == 1) {  // lowpass
	dbg = 0.0;
	g1 = sqrt(pow(10.0, (dbg / 20.0)));
        w0 = 2.0 * 3.14159265358979 * cf / sr;
	alpha = sin(w0)/(2.0 * q);
	if (ftype == 1) y = 1.0 - cos(w0);
	else            y = 1.0 + cos(w0);
	b0 =  y / 2.0;
	b1 =  y;
	b2 =  y / 2.0;
	a0 =  1.0 + alpha;
	a1 = -2.0 * cos(w0);
	a2 =  1.0 - alpha;
    }
    p->a0 = a0;
    p->a1 = a1/a0;
    p->a2 = a2/a0;
    p->b0 = b0/a0;
    p->b1 = b1/a0;
    p->b2 = b2/a0;
    p->ys_1L =  0.0;
    p->ys_0L =  0.0;
    p->xs_1L =  0.0;
    p->xs_0L =  0.0;
    p->ys_1R =  0.0;
    p->ys_0R =  0.0;
    p->xs_1R =  0.0;
    p->xs_0R =  0.0;
    p->sr    =  sr;
    p->cf    =  cf;
    p->q     =  q;
    p->ftype =  ftype;
}

// iir float biquad cascade
// butterworth biquad cascade
double bbcascade[36] = {
  0.70710678, 0.0,0.0, 0.0,0.0,0.0,
  0.54119610, 1.3065630, 0.0, 0.0,0.0,0.0,
  0.51763809, 0.70710678, 1.9318517, 0.0,0.0,0.0,
  0.50979558, 0.60134489, 0.89997622, 2.5629154, 0.0, 0.0,
  0.50623256, 0.56116312, 0.70710678, 1.1013446, 3.1962266, 0.0,
  0.50431448, 0.54119610, 0.63023621, 0.82133982, 1.3065630, 3.8306488
};

struct iirParams ipbc[36];

void init_ipbc(double sr, double cf)
{
    int i,j;
    for (j=0;j<6;j++) {
       for (i=0;i<6;i++) {
	  int ftype = 0;
	  int k = 6*j + i;
	  iirParams *p = &ipbc[k];
	  float q = bbcascade[k];
	  if (q > 0.0) { ftype = 1; }	// low pass
	  calc_iir_coefs(ftype, cf, q, sr, p);
       }
    }
}

void iir_fbc(float *s, int n, int order)
{
    int num_biquads = order / 2;
    int k = 0;
    int batch = 4096; // 16384 fits in dcache
    while (k < n) {
        int m = batch;
	if (k + batch > n) { m = n - k; }
        for (int b=0;b<6;b++) {
            int j = 6*(num_biquads-1) + b;
            iirParams *p = &ipbc[j];
            if (p->ftype == 1) {
	        iir_f2(&s[k], m, p);
            }
        }
	k += batch;
    }
}

void init_iir()
{
    double        sr, bw;
    sr =  192000.0;
    bw =   16000.0;
    	// int type = 1; // lowpass
        // call calc_iir_coefs(type, bw, q, sr, &pp);
        //   with 6 sets of 6 coeffs for 2nd to 12th order filtering
    init_ipbc(sr, bw);
        // int order =  12;		// set filter order
        // iir_fbc(&uu[0], n, order);
}

/* eof */

// eof
