/*-
 * Copyright (c) 2014-2016 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <sys/endian.h>
#include <sys/soundcard.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <signal.h>
#include <sysexits.h>
#include <stdlib.h>
#include <string.h>
#include <sys/fcntl.h>
#include <sys/mman.h>
#include <sys/queue.h>
#include <unistd.h>
#include <err.h>
#include <libv4l2.h>

#ifdef HAVE_X11_SUPPORT
#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#endif

#define	NBUFFER 4
#define	NVIDEODEV 16
#define	NAUDIODEV 16

struct data;
typedef TAILQ_HEAD(,data) head_t;
struct data {
	TAILQ_ENTRY(data) entry;
	int fd;
	int bytes;
	uint8_t data[0];
};

static head_t data_head = TAILQ_HEAD_INITIALIZER(data_head);

/* the following table was imported from ffmpeg's V4L2 driver */

static const struct {
	const char *const pixfmt_str;
	const char *const codec_str;
	uint32_t pixfmt;
} format_table[] = {
	{ "yuv420p", "rawvideo", V4L2_PIX_FMT_YUV420 },
	{ "yuv420p", "rawvideo", V4L2_PIX_FMT_YVU420 },
	{ "yuv422p", "rawvideo", V4L2_PIX_FMT_YUV422P },
	{ "yuyv422", "rawvideo", V4L2_PIX_FMT_YUYV },
	{ "uyvy422", "rawvideo", V4L2_PIX_FMT_UYVY },
	{ "yuv411p", "rawvideo", V4L2_PIX_FMT_YUV411P },
	{ "yuv410p", "rawvideo", V4L2_PIX_FMT_YUV410 },
	{ "yuv410p", "rawvideo", V4L2_PIX_FMT_YVU410 },
	{ "rgb555le", "rawvideo", V4L2_PIX_FMT_RGB555 },
	{ "rgb555be", "rawvideo", V4L2_PIX_FMT_RGB555X },
	{ "rgb565le", "rawvideo", V4L2_PIX_FMT_RGB565 },
	{ "rgb565be", "rawvideo", V4L2_PIX_FMT_RGB565X },
	{ "bgr24", "rawvideo", V4L2_PIX_FMT_BGR24 },
	{ "rgb24", "rawvideo", V4L2_PIX_FMT_RGB24 },
	{ "bgr0", "rawvideo", V4L2_PIX_FMT_BGR32 },
	{ "0rgb", "rawvideo", V4L2_PIX_FMT_RGB32 },
	{ "gray8", "rawvideo", V4L2_PIX_FMT_GREY },
	{ "gray16le", "rawvideo", V4L2_PIX_FMT_Y16 },
	{ "nv12", "rawvideo", V4L2_PIX_FMT_NV12 },
	{ "", "mjpeg", V4L2_PIX_FMT_MJPEG },
	{ "", "mjpeg", V4L2_PIX_FMT_JPEG },
	{ "", "h264", V4L2_PIX_FMT_H264 },
	{ "", "cpia", V4L2_PIX_FMT_CPIA1 },
	{ "bayer_bggr8", "rawvideo", V4L2_PIX_FMT_SBGGR8 },
	{ "bayer_gbrg8", "rawvideo", V4L2_PIX_FMT_SGBRG8 },
	{ "bayer_grbg8", "rawvideo", V4L2_PIX_FMT_SGRBG8 },
	{ "bayer_rggb8", "rawvideo", V4L2_PIX_FMT_SRGGB8 },
	{ NULL, NULL, 0 }
};

struct vc_info {
	void   *framebuffer;
	uint32_t framebytesused;
	uint32_t framesize;
	const char *devname;
	void   *addr[NBUFFER];
	int	fd;
	int	pipe;
	int	pid;
	uint16_t width;
	uint16_t height;
	struct v4l2_capability cap;
	struct v4l2_format fmt;
	struct v4l2_buffer buf;
	struct v4l2_requestbuffers rb;
};

struct ac_info {
	void   *framebuffer;
	uint32_t framesize;
	const char *devname;
	double	fps;
	int	fd;
	int	out_fd;
	uint32_t channels;
	uint32_t bits;
	uint32_t speed;
	off_t	bytes;
};

static struct vc_info vc_info[NVIDEODEV];
static struct ac_info ac_info[NAUDIODEV];
static pthread_mutex_t atomic_mtx;
static pthread_cond_t atomic_cv;
static int nvideo;
static int naudio;
static int wait_init;
static int default_width = 640;
static int default_height = 480;
static int default_rate = 48000;
static int default_blocksize = 2000;	/* 24 FPS is default */
static const char *default_prefix = "project";
static int default_quality = -1;

static int write_le32(int, uint32_t);

static void
atomic_lock(void)
{
	pthread_mutex_lock(&atomic_mtx);
}

static void
atomic_unlock(void)
{
	pthread_mutex_unlock(&atomic_mtx);
}

static void
do_atexit(void)
{
	off_t off;
	int i;
	int f;

	for (i = 0; i != nvideo; i++) {
		if (vc_info[i].pid != 0)
			kill(vc_info[i].pid, SIGKILL);
	}
}

static int
write_async_locked(int fd, const void *data, int bytes)
{
	struct data *ptr;
	ptr = malloc(sizeof(*ptr) + bytes);
	if (ptr == NULL)
		errx(EX_SOFTWARE, "Out of memory");
	ptr->fd = fd;
	ptr->bytes = bytes;
	memcpy(ptr + 1, data, bytes);
	TAILQ_INSERT_TAIL(&data_head, ptr, entry);
	pthread_cond_signal(&atomic_cv);
	return (bytes);
}

static void
close_fds(void)
{
	int i;

	for (i = 0; i != nvideo; i++) {
		if (vc_info[i].fd > 0)
			close(vc_info[i].fd);
		if (vc_info[i].pipe > 0)
			close(vc_info[i].pipe);
	}
	for (i = 0; i != naudio; i++) {
		if (ac_info[i].fd > 0)
			close(ac_info[i].fd);
		if (ac_info[i].out_fd > 0)
			close(ac_info[i].out_fd);
	}
}

static int
create_piped_process(const char *cmd, int *ppid)
{
	int fds[2];

	if (pipe(fds) != 0)
		errx(EX_SOFTWARE, "Could not create pipe");

	if ((*ppid = fork()) == 0) {
		close_fds();
		dup2(fds[0], 0);
		close(fds[1]);
		close(fds[0]);
		system(cmd);
		exit(0);
	} else {
		close(fds[0]);
	}

	return (fds[1]);
}

#ifdef HAVE_X11_SUPPORT
static void *
x11_thread(void *arg)
{
	struct vc_info *pcvi = arg;
	XWindowAttributes win_info;
	Window window = strtoul(pcvi->devname, NULL, 0);
	const char *display_name = NULL;
	Display *dpy;
	int screen;
	int do_unref = 1;
	int us_delay;

	dpy = XOpenDisplay(display_name);
	if (dpy == NULL)
		errx(1, "Unable to open display '%s'", XDisplayName(display_name));
	screen = XDefaultScreen(dpy);

	if (XGetWindowAttributes(dpy, window, &win_info) == 0)
		errx(1, "Unable to get target window attributes");

	pcvi->width = win_info.width;
	pcvi->height = win_info.height;

	if (default_blocksize <= 0)
		errx(1, "Blocksize is invalid");
	if (default_rate <= 0)
		errx(1, "Samplerate is invalid");

	/* compute a reasonable delay to wait for frame capturing */
	us_delay = default_rate / default_blocksize;
	if (us_delay < 1)
		us_delay = 1;
	us_delay = 1000000 / us_delay;
	us_delay = us_delay - (us_delay >> 4);
	if (us_delay < 1)
		us_delay = 1;

	while (1) {
		Window dummy;
		size_t buffer_size;
		XImage *image;
		int absx, absy;

		usleep(us_delay);

		if (!XTranslateCoordinates(dpy, window, RootWindow(dpy, screen), 0, 0,
		    &absx, &absy, &dummy))
			continue;
		win_info.x = absx;
		win_info.y = absy;

		image = XGetImage(dpy, RootWindow(dpy, screen), win_info.x, win_info.y,
		    win_info.width, win_info.height, AllPlanes, ZPixmap);
		if (image == NULL)
			continue;

		buffer_size = (image->bytes_per_line * image->height);

		atomic_lock();
		if (do_unref != 0) {
			switch (image->bits_per_pixel) {
			case 32:
				pcvi->fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR32;
				break;
			case 24:
				pcvi->fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
				break;
			case 16:
				pcvi->fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565X;
				break;
			case 15:
				pcvi->fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB555X;
				break;
			default:
				errx(1, "Unsupported pixel format");
			}

			pcvi->framesize = ((image->bits_per_pixel + 7) / 8) *
			    pcvi->width * pcvi->height;
			pcvi->framebuffer = malloc(pcvi->framesize);
			if (pcvi->framebuffer == NULL)
				errx(EX_SOFTWARE, "Cannot allocate memory");
			memset(pcvi->framebuffer, 0, pcvi->framesize);
		}
		if (buffer_size == pcvi->framesize) {
			memcpy(pcvi->framebuffer, image->data, buffer_size);
			pcvi->framebytesused = buffer_size;
		} else {
			pcvi->framebytesused = 0;
		}
		if (do_unref) {
			wait_init--;
			do_unref = 0;
		}
		atomic_unlock();

		XDestroyImage(image);
	}
	return (NULL);
}

#endif

static void *
video_thread(void *arg)
{
	struct vc_info *pcvi = arg;
	int error;
	int i;
	int do_unref = 1;

	/* standard V4L2 setup */

	pcvi->fd = v4l2_open(pcvi->devname, O_RDWR);
	if (pcvi->fd < 0)
		errx(EX_SOFTWARE, "Cannot open device '%s'", pcvi->devname);

	error = v4l2_ioctl(pcvi->fd, VIDIOC_QUERYCAP, &pcvi->cap);
	if (error != 0)
		errx(EX_SOFTWARE, "%s: Cannot query capabilities", pcvi->devname);

	if (!(pcvi->cap.capabilities & V4L2_CAP_STREAMING))
		errx(EX_SOFTWARE, "%s: Device doesn't support mmap", pcvi->devname);

	pcvi->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	pcvi->fmt.fmt.pix.width = pcvi->width;
	pcvi->fmt.fmt.pix.height = pcvi->height;
	pcvi->fmt.fmt.pix.field = V4L2_FIELD_ANY;

	/* try all formats available */
	for (error = -1, i = 0; format_table[i].codec_str != NULL; i++) {
		pcvi->fmt.fmt.pix.pixelformat = format_table[i].pixfmt;
		error = v4l2_ioctl(pcvi->fd, VIDIOC_S_FMT, &pcvi->fmt);
		if (error == 0)
			break;
	}
	if (error != 0)
		errx(EX_SOFTWARE, "%s: Cannot set format", pcvi->devname);

	error = v4l2_ioctl(pcvi->fd, VIDIOC_G_FMT, &pcvi->fmt);
	if (error != 0)
		errx(EX_SOFTWARE, "%s: Cannot get format", pcvi->devname);

	pcvi->width = pcvi->fmt.fmt.pix.width;
	pcvi->height = pcvi->fmt.fmt.pix.height;

	pcvi->rb.count = NBUFFER;
	pcvi->rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	pcvi->rb.memory = V4L2_MEMORY_MMAP;

	error = v4l2_ioctl(pcvi->fd, VIDIOC_REQBUFS, &pcvi->rb);
	if (error != 0)
		errx(EX_SOFTWARE, "%s: Cannot request buffers", pcvi->devname);

	for (i = 0; i != NBUFFER; i++) {
		pcvi->buf.index = i;
		pcvi->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		pcvi->buf.memory = V4L2_MEMORY_MMAP;
		error = v4l2_ioctl(pcvi->fd, VIDIOC_QUERYBUF, &pcvi->buf);
		if (error != 0)
			errx(EX_SOFTWARE, "%s: Cannot query buffer", pcvi->devname);
		pcvi->framesize = pcvi->buf.length;
		pcvi->addr[i] = v4l2_mmap(0,
		    pcvi->buf.length, PROT_READ, MAP_SHARED, pcvi->fd,
		    pcvi->buf.m.offset);
		if (pcvi->addr[i] == MAP_FAILED)
			errx(EX_SOFTWARE, "%s: Cannot mmap buffer", pcvi->devname);
	}
	for (i = 0; i != NBUFFER; i++) {
		pcvi->buf.index = i;
		pcvi->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		pcvi->buf.memory = V4L2_MEMORY_MMAP;
		error = v4l2_ioctl(pcvi->fd, VIDIOC_QBUF, &pcvi->buf);
		if (error != 0)
			errx(EX_SOFTWARE, "%s: Cannot queue buffer", pcvi->devname);
	}

	pcvi->framebuffer = malloc(pcvi->framesize);
	if (pcvi->framebuffer == NULL)
		errx(EX_SOFTWARE, "%s: Cannot allocate memory", pcvi->devname);

	memset(pcvi->framebuffer, 0, pcvi->framesize);

	i = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	error = v4l2_ioctl(pcvi->fd, VIDIOC_STREAMON, &i);
	if (error != 0)
		errx(EX_SOFTWARE, "%s: Cannot enable stream", pcvi->devname);

	while (1) {
		pcvi->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		pcvi->buf.memory = V4L2_MEMORY_MMAP;
		error = v4l2_ioctl(pcvi->fd, VIDIOC_DQBUF, &pcvi->buf);
		if (error != 0)
			errx(EX_SOFTWARE, "%s: Cannot dequeue buffer", pcvi->devname);

		atomic_lock();
		if ((uint32_t)pcvi->buf.bytesused <= pcvi->framesize) {
			memcpy(pcvi->framebuffer,
			    pcvi->addr[pcvi->buf.index], pcvi->buf.bytesused);
			pcvi->framebytesused = pcvi->buf.bytesused;
		} else {
			pcvi->framebytesused = 0;
		}
		if (do_unref != 0) {
			wait_init--;
			do_unref = 0;
		}
		atomic_unlock();

		error = v4l2_ioctl(pcvi->fd, VIDIOC_QBUF, &pcvi->buf);
		if (error != 0)
			errx(EX_SOFTWARE, "%s: Cannot enqueue buffer", pcvi->devname);
	}
	return (NULL);
}

static int
write_le32(int fd, uint32_t val)
{
	val = htole32(val);
	return (write(fd, &val, sizeof(val)));
}

static int
write_le16(int fd, uint16_t val)
{
	val = htole16(val);
	return (write(fd, &val, sizeof(val)));
}

static void *
audio_thread(void *arg)
{
	struct ac_info *pcai = arg;
	int error;
	int i;

	atomic_lock();
	wait_init--;
	while (wait_init != 0) {
		atomic_unlock();
		usleep(1000);
		atomic_lock();
	}
	atomic_unlock();

	pcai->fd = open(pcai->devname, O_RDONLY);
	if (pcai->fd < 0)
		errx(EX_SOFTWARE, "Cannot open device '%s'", pcai->devname);

	if (1) {
		char fname[128];

		snprintf(fname, sizeof(fname), "%s_audio_%d.wav", default_prefix, (int)(pcai - ac_info));

		pcai->out_fd = open(fname, O_WRONLY | O_TRUNC | O_CREAT, 0644);
		if (pcai->out_fd < 0)
			errx(EX_SOFTWARE, "Cannot open device '%s'", fname);
	}
	i = 0;
	error = ioctl(pcai->fd, FIONBIO, &i);
	if (error != 0)
		errx(EX_SOFTWARE, "%s: Cannot set blocking behaviour", pcai->devname);

	i = 0;
	error = ioctl(pcai->fd, SNDCTL_DSP_GETFMTS, &i);
	if (error != 0)
		errx(EX_SOFTWARE, "%s: Cannot get format", pcai->devname);
	if (i & AFMT_S32_LE) {
		pcai->bits = 32;
		i = AFMT_S32_LE;
	} else if (i & AFMT_S24_LE) {
		pcai->bits = 24;
		i = AFMT_S24_LE;
	} else if (i & AFMT_S16_LE) {
		pcai->bits = 16;
		i = AFMT_S16_LE;
	} else if (i & AFMT_S8) {
		pcai->bits = 8;
		i = AFMT_S8;
	} else {
		errx(EX_SOFTWARE, "%s: No supported formats", pcai->devname);
	}
	error = ioctl(pcai->fd, SNDCTL_DSP_SETFMT, &i);
	if (error != 0)
		errx(EX_SOFTWARE, "%s: Cannot set format", pcai->devname);

	i = 2;
	error = ioctl(pcai->fd, SOUND_PCM_READ_CHANNELS, &i);
	if (error != 0)
		errx(EX_SOFTWARE, "%s: Cannot set read channels", pcai->devname);

	pcai->channels = i;

	i = default_rate;
	error = ioctl(pcai->fd, SNDCTL_DSP_SPEED, &i);
	if (error != 0 || i != default_rate)
		errx(EX_SOFTWARE, "%s: Cannot set sample rate", pcai->devname);

	pcai->speed = i;

	i = pcai->channels * default_blocksize;
	if (i < 1)
		errx(EX_SOFTWARE, "%s: Invalid block size %d", pcai->devname, i);
	error = ioctl(pcai->fd, SNDCTL_DSP_SETBLKSIZE, &i);
	pcai->framesize = i * (pcai->bits / 8);
	pcai->framebuffer = malloc(i);
	if (pcai->framebuffer == NULL)
		errx(EX_SOFTWARE, "%s: Cannot allocate buffer", pcai->devname);

	if (pcai == ac_info) {
		pcai->fps = (double)default_rate / (double)default_blocksize;
		for (i = 0; i != nvideo; i++) {
			char cmdbuf[256];
			char sizebuf[128];
			char fmtbuf[128];
			int j;

			for (j = 0; format_table[j].codec_str != NULL; j++) {
				if (format_table[j].pixfmt == vc_info[i].fmt.fmt.pix.pixelformat)
					break;
			}
			if (format_table[j].codec_str == NULL)
				errx(EX_SOFTWARE, "%s: Unsupported V4L2 pixel format", vc_info[i].devname);

			if (format_table[j].pixfmt_str[0] != 0) {
				snprintf(sizebuf, sizeof(sizebuf), "-video_size %dx%d -pix_fmt %s",
				    (int)vc_info[i].width, (int)vc_info[i].height,
				    format_table[j].pixfmt_str);
			} else {
				sizebuf[0] = 0;
			}

			if (default_quality < 0)
				snprintf(fmtbuf, sizeof(fmtbuf), "-vcodec huffyuv");
			else
				snprintf(fmtbuf, sizeof(fmtbuf), "-qscale %d -vcodec mjpeg", default_quality);

			snprintf(cmdbuf, sizeof(cmdbuf), "ffmpeg -loglevel quiet -f %s %s -framerate %f -r %f "
			    " -i /dev/stdin %s -vsync drop -start_at_zero -f matroska -y '%s_camera_%d.mkv'",
			    format_table[j].codec_str, sizebuf, (float)pcai->fps,
			    (float)pcai->fps, fmtbuf, default_prefix, i);
			printf("CMD: %s\n", cmdbuf);
			vc_info[i].pipe = create_piped_process(cmdbuf, &vc_info[i].pid);
		}
	}
	if (write(pcai->out_fd, "RIFF", 4) != 4 ||
	    write_le32(pcai->out_fd, 36 - 8) != 4 ||
	    write(pcai->out_fd, "WAVEfmt ", 8) != 8 ||
	    write_le32(pcai->out_fd, 16U) != 4 ||
	    write_le16(pcai->out_fd, 1U) != 2 ||
	    write_le16(pcai->out_fd, pcai->channels) != 2 ||
	    write_le32(pcai->out_fd, pcai->speed) != 4 ||
	    write_le32(pcai->out_fd, pcai->speed * pcai->channels * (pcai->bits / 8)) != 4 ||
	    write_le16(pcai->out_fd, pcai->channels * (pcai->bits / 8)) != 2 ||
	    write_le16(pcai->out_fd, pcai->bits) != 2 ||
	    write(pcai->out_fd, "data", 4) != 4 ||
	    write_le32(pcai->out_fd, 0x7ffff000U /* unspecified length */ ) != 4)
		errx(EX_SOFTWARE, "%s: Could not write WAV header", pcai->devname);

	pcai->bytes += 44;

	while (1) {
		error = read(pcai->fd, pcai->framebuffer, pcai->framesize);
		if (error != pcai->framesize)
			errx(EX_SOFTWARE, "%s: Could not read from DSP device", pcai->devname);

		atomic_lock();
		if (pcai == ac_info) {
			for (i = 0; i != nvideo; i++) {
				if (vc_info[i].framebytesused == 0)
					continue;
				error = write_async_locked(vc_info[i].pipe,
				    vc_info[i].framebuffer,
				    vc_info[i].framebytesused);
				if (error != (int)vc_info[i].framebytesused)
					errx(EX_SOFTWARE, "%s: Could not write to pipe", pcai->devname);
			}
		}
		error = write_async_locked(pcai->out_fd, pcai->framebuffer, pcai->framesize);
		if (error != pcai->framesize)
			errx(EX_SOFTWARE, "%s: Could not write to file", pcai->devname);

		pcai->bytes += pcai->framesize;
		atomic_unlock();

		if (pcai->bytes >= (1ULL << 31))
			errx(EX_SOFTWARE, "%s: File too big", pcai->devname);
	}
	return (NULL);
}

static void
usage(void)
{
	fprintf(stderr, "usage: videocast [-q 0] [-w 640] [-h 480] [-b 8000] "
	    "[-r 96000] [-p prefix] [-v /dev/video0] [-v 0xXXXXXXXX] -d /dev/dsp\n"
	    "[-b 3675] [-r 44100] [-b 2000] [-r 48000]\n");
	exit(0);
}

int
main(int argc, char **argv)
{
	int c;

	pthread_mutex_init(&atomic_mtx, NULL);
	pthread_cond_init(&atomic_cv, NULL);

	while ((c = getopt(argc, argv, "h:w:v:d:p:r:b:q:")) != -1) {
		switch (c) {
		case 'b':
			default_blocksize = atoi(optarg);
			break;
		case 'w':
			default_width = atoi(optarg);
			break;
		case 'h':
			default_height = atoi(optarg);
			break;
		case 'r':
			default_rate = atoi(optarg);
			break;
		case 'p':
			default_prefix = optarg;
			break;
		case 'v':
			if (nvideo == NVIDEODEV)
				errx(EX_SOFTWARE, "Too many video devices");
			vc_info[nvideo].devname = optarg;
			vc_info[nvideo].width = default_width;
			vc_info[nvideo].height = default_height;
			nvideo++;
			break;
		case 'd':
			if (naudio == NAUDIODEV)
				errx(EX_SOFTWARE, "Too many audio devices");
			ac_info[naudio].devname = optarg;
			naudio++;
			break;
		case 'q':
			default_quality = atoi(optarg);
			break;
		default:
			usage();
			break;
		}
	}

	if (nvideo == 0 || naudio == 0)
		usage();

	atexit(&do_atexit);

	wait_init = naudio + nvideo;

	for (c = 0; c != nvideo; c++) {
		pthread_t dummy;

		if (isdigit(vc_info[c].devname[0])) {
#ifdef HAVE_X11_SUPPORT
			if (pthread_create(&dummy, NULL, &x11_thread, vc_info + c))
				errx(EX_SOFTWARE, "Couldn't create thread");
#else
			errx(EX_SOFTWARE, "X11 support not compiled");
#endif
		} else {
			if (pthread_create(&dummy, NULL, &video_thread, vc_info + c))
				errx(EX_SOFTWARE, "Couldn't create thread");
		}
	}

	for (c = 0; c != naudio; c++) {
		pthread_t dummy;

		if (pthread_create(&dummy, NULL, &audio_thread, ac_info + c))
			errx(EX_SOFTWARE, "Couldn't create thread");
	}

	atomic_lock();
	while (wait_init != 0) {
		atomic_unlock();
		usleep(1000);
		atomic_lock();
	}
	atomic_unlock();

	printf("Press CTRL+C to complete recording\n");

	atomic_lock();
	while (1) {
		struct data *ptr;
		ptr = TAILQ_FIRST(&data_head);
		if (ptr == NULL) {
			pthread_cond_wait(&atomic_cv, &atomic_mtx);
			continue;
		}
		TAILQ_REMOVE(&data_head, ptr, entry);
		atomic_unlock();
		if (write(ptr->fd, ptr->data, ptr->bytes) != ptr->bytes)
			errx(EX_SOFTWARE, "Could not write data to file");
		free(ptr);
		atomic_lock();
	}
	atomic_unlock();	
	return (0);
}
