OS := $(shell uname)
HH = /usr/local/include/libairspyhf 

ifeq ($(OS), Darwin)
	CC  = clang
else
ifeq ($(OS), Linux)
	CC  = cc
	STD = -std=c99
else
	$(error OS not detected)
endif
endif

hfp_tcp:	hfp_tcp_server.c
		$(info Building for $(OS))
		$(CC) -I$(HH) hfp_tcp_server.c -o hfp_tcp $(STD) -lm -lairspyhf

install:	hfp_tcp
		cp ./hfp_tcp /usr/local/bin

clean: 
	rm hfp_tcp
