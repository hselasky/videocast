PREFIX?=        /usr/local
PTHREAD_LIBS?=	-lpthread
LOCALBASE?=     /usr/local
BINDIR=         ${PREFIX}/sbin
PROG=videocast
MAN=
CFLAGS=-I${LOCALBASE}/include
SRCS+= videocast.c
LDFLAGS+= ${PTHREAD_LIBS} -L${LOCALBASE}/lib -lv4l2
.if defined(HAVE_X11_SUPPORT)
LDFLAGS+= -lX11
CFLAGS+= -DHAVE_X11_SUPPORT
.endif

.include <bsd.prog.mk>
