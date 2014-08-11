PTHREAD_LIBS?=	-lpthread
LOCALBASE?=     /usr/local

PROG=videocast
MAN=
CFLAGS=-I${LOCALBASE}/include
SRCS+= videocast.c
LDFLAGS+= ${PTHREAD_LIBS}

.include <bsd.prog.mk>
