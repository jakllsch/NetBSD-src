#	$NetBSD: Makefile,v 1.11 2020/06/07 23:26:54 fox Exp $

# $FreeBSD: head/cddl/usr.bin/ctfmerge/Makefile 314654 2017-03-04 11:30:04Z ngie $

.include <bsd.own.mk>

.include	"../../Makefile.inc"
.include	"../../Makefile.ctf"

.PATH:          ${OPENSOLARIS_USR_DISTDIR}/tools/ctf/cvt

PROG=		ctfmerge

SRCS=		alist.c \
		barrier.c \
		ctf.c \
		ctfmerge.c \
		fifo.c \
		hash.c \
		iidesc.c \
		input.c \
		list.c \
		memory.c \
		merge.c \
		output.c \
		strtab.c \
		symbol.c \
		tdata.c \
		traverse.c \
		util.c

COPTS.util.c +=		-Wno-format-nonliteral

.if (${HOSTPROG:U} == "")
LDADD+=-lpthread
DPADD+=${LIBPTHREAD}
.endif

COPTS.ctfmerge.c+=	${GCC_NO_STRINGOP_TRUNCATION}

.include <bsd.prog.mk>
