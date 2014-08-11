PREFIX?=        /usr/local
PTHREAD_LIBS?=	-lpthread
LOCALBASE?=     /usr/local
BINDIR=         ${PREFIX}/sbin
PROG=videocast
MAN=
CFLAGS=-I${LOCALBASE}/include
SRCS+= videocast.c
LDFLAGS+= ${PTHREAD_LIBS}

.include <bsd.prog.mk>
