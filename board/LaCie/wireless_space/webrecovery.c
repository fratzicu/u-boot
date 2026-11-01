// SPDX-License-Identifier: GPL-2.0+
/*
 * Minimal web recovery server for the LaCie Wireless Space.
 *
 * "httpd" serves an upload page on port 80 (lwIP raw TCP API). A firmware
 * image POSTed by the browser is stored in RAM at $loadaddr, the multipart
 * envelope is stripped, and the command returns with:
 *	filesize     = image size (hex)
 *	filesize_blk = size rounded up to the 128 KiB NAND erase block, the
 *		       padding filled with 0xff (for "nand write")
 *	img_type     = ubi | uimage | kwb | unknown
 * The environment ("recovery_apply") then decides what to do with it.
 *
 * "imgtype <addr> <size>" runs the same detection on an image loaded by
 * other means (USB, TFTP, wget...).
 */

#include <command.h>
#include <console.h>
#include <env.h>
#include <mapmem.h>
#include <net.h>
#include <time.h>
#include <version.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <lwip/tcp.h>

#define HTTPD_PORT		80
#define HTTPD_HDR_MAX		2048
#define HTTPD_MAX_UPLOAD	(120 << 20)
#define NAND_BLOCK_SIZE		0x20000

enum httpd_state {
	HTTPD_RUNNING,
	HTTPD_UPLOADED,
	HTTPD_FAILED,
};

struct httpd_conn {
	char hdr[HTTPD_HDR_MAX + 1];
	int hdr_len;
	bool hdr_done;
	bool is_upload;
	char boundary[72];
	ulong content_len;
	ulong received;
	const char *tx;		/* response still to send */
	int tx_left;
	char *tx_buf;		/* allocated response, freed when sent */
	bool close_when_sent;
	bool finish_when_sent;	/* upload complete: stop the server */
};

static enum httpd_state httpd_state;
static ulong httpd_addr;
static ulong httpd_size;

static const char page_head[] =
	"<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
	"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
	"<title>Wireless Space recovery</title><style>"
	"body{font-family:sans-serif;max-width:40em;margin:2em auto;padding:0 1em;"
	"background:#f4f4f4;color:#222}"
	"div{background:#fff;border-radius:8px;padding:1em 1.5em;margin:1em 0;"
	"box-shadow:0 1px 3px #0002}"
	"h1{font-size:1.4em}code{background:#eee;padding:0 .3em}"
	"input[type=submit]{font-size:1em;padding:.5em 1.5em}"
	".w{color:#a00}</style></head><body>"
	"<h1>LaCie Wireless Space &mdash; recovery</h1>";

static const char page_form[] =
	"<div><form method=\"post\" action=\"/upload\" "
	"enctype=\"multipart/form-data\">"
	"<p>Firmware file:</p><p><input type=\"file\" name=\"firmware\"></p>"
	"<p><input type=\"submit\" value=\"Upload\"></p></form></div>"
	"<div><p>Accepted images:</p><ul>"
	"<li><code>*-factory.bin</code> &mdash; OpenWrt UBI image, written to NAND</li>"
	"<li><code>*-initramfs-uImage</code> &mdash; booted from RAM, nothing is written</li>"
	"<li><code>u-boot.kwb</code> &mdash; new bootloader, written to NAND "
	"<span class=\"w\">(a bad one needs serial/JTAG to recover)</span></li>"
	"</ul><p>Do not power off while writing.</p></div>";

static const char page_tail[] = "</body></html>";

static const char *img_type_desc(const char *type)
{
	if (!strcmp(type, "ubi"))
		return "OpenWrt UBI image: writing to NAND, the device reboots when done.";
	if (!strcmp(type, "uimage"))
		return "Kernel image: booting it from RAM.";
	if (!strcmp(type, "kwb"))
		return "U-Boot image: writing the bootloader, the device reboots when done.";
	return "Unknown image type: nothing was written. Go back and try another file.";
}

/* Same detection for every source: httpd, USB, TFTP, wget */
static const char *detect_img_type(const u8 *p, ulong size)
{
	if (size >= 64 && !memcmp(p, "UBI#", 4))
		return "ubi";
	if (size >= 64 && p[0] == 0x27 && p[1] == 0x05 && p[2] == 0x19 &&
	    p[3] == 0x56)
		return "uimage";
	if (size >= 32 && p[0] == 0x8b) {
		/* kwbimage v0: byte 0x1f is the 8-bit sum of bytes 0x00..0x1e */
		u8 csum = 0;
		int i;

		for (i = 0; i < 0x1f; i++)
			csum += p[i];
		if (csum == p[0x1f])
			return "kwb";
	}
	return "unknown";
}

static void set_img_env(ulong addr, ulong size, const char *type)
{
	ulong blk = ALIGN(size, NAND_BLOCK_SIZE);
	u8 *p = map_sysmem(addr, blk);

	memset(p + size, 0xff, blk - size);
	unmap_sysmem(p);

	env_set_hex("filesize", size);
	env_set_hex("filesize_blk", blk);
	env_set("img_type", type);
}

static void httpd_send_more(struct tcp_pcb *pcb, struct httpd_conn *c)
{
	while (c->tx_left > 0) {
		int n = min_t(int, c->tx_left, tcp_sndbuf(pcb));

		if (n <= 0 || tcp_write(pcb, c->tx, n, TCP_WRITE_FLAG_COPY) != ERR_OK)
			break;
		c->tx += n;
		c->tx_left -= n;
	}
	tcp_output(pcb);
}

static void httpd_conn_free(struct httpd_conn *c)
{
	free(c->tx_buf);
	free(c);
}

static void httpd_close(struct tcp_pcb *pcb, struct httpd_conn *c)
{
	tcp_arg(pcb, NULL);
	tcp_recv(pcb, NULL);
	tcp_sent(pcb, NULL);
	tcp_err(pcb, NULL);
	if (c->finish_when_sent)
		httpd_state = HTTPD_UPLOADED;
	httpd_conn_free(c);
	if (tcp_close(pcb) != ERR_OK)
		tcp_abort(pcb);
}

static void httpd_respond(struct tcp_pcb *pcb, struct httpd_conn *c,
			  const char *status, const char *body, bool finish)
{
	static const char fmt[] = "HTTP/1.0 %s\r\n"
		"Content-Type: text/html; charset=utf-8\r\n"
		"Cache-Control: no-store\r\nConnection: close\r\n"
		"Content-Length: %d\r\n\r\n%s%s%s";
	int len = strlen(page_head) + strlen(body) + strlen(page_tail);
	int size = sizeof(fmt) + 64 + len;

	c->tx_buf = malloc(size);
	if (!c->tx_buf) {
		/* drop the request; closed when the peer gives up */
		c->close_when_sent = true;
		return;
	}
	snprintf(c->tx_buf, size, fmt, status, len, page_head, body, page_tail);
	c->tx = c->tx_buf;
	c->tx_left = strlen(c->tx_buf);
	c->close_when_sent = true;
	c->finish_when_sent = finish;
	httpd_send_more(pcb, c);
}

static err_t httpd_sent_cb(void *arg, struct tcp_pcb *pcb, u16_t len)
{
	struct httpd_conn *c = arg;

	if (!c)
		return ERR_OK;
	if (c->tx_left > 0)
		httpd_send_more(pcb, c);
	else if (c->close_when_sent)
		httpd_close(pcb, c);
	return ERR_OK;
}

static void httpd_err_cb(void *arg, err_t err)
{
	struct httpd_conn *c = arg;

	/* the pcb is already freed by lwIP */
	if (c) {
		if (c->finish_when_sent)
			httpd_state = HTTPD_UPLOADED;
		httpd_conn_free(c);
	}
}

/* Strip the multipart/form-data envelope around the uploaded file */
static int httpd_extract_file(struct httpd_conn *c)
{
	u8 *body = map_sysmem(httpd_addr, c->received);
	ulong blen = strlen(c->boundary);
	u8 *start = NULL, *end = NULL;
	ulong i, scan;

	/* part headers end with an empty line, within the first 2 KiB */
	scan = min_t(ulong, c->received, 2048);
	for (i = 0; i + 4 <= scan; i++) {
		if (!memcmp(body + i, "\r\n\r\n", 4)) {
			start = body + i + 4;
			break;
		}
	}
	/* closing "\r\n--boundary" is in the last bytes */
	if (start && c->received > blen + 4) {
		for (i = c->received - blen - 4; body + i >= start; i--) {
			if (body[i] == '\r' && body[i + 1] == '\n' &&
			    body[i + 2] == '-' && body[i + 3] == '-' &&
			    !memcmp(body + i + 4, c->boundary, blen)) {
				end = body + i;
				break;
			}
			if (c->received - i > 512)
				break;
		}
	}
	if (!start || !end || end <= start) {
		unmap_sysmem(body);
		return -EINVAL;
	}

	httpd_size = end - start;
	memmove(body, start, httpd_size);
	unmap_sysmem(body);
	return 0;
}

static void httpd_upload_done(struct tcp_pcb *pcb, struct httpd_conn *c)
{
	char msg[512];
	const char *type;
	u8 *img;

	if (httpd_extract_file(c)) {
		httpd_respond(pcb, c, "400 Bad Request",
			      "<div><p class=\"w\">Malformed upload.</p></div>",
			      false);
		return;
	}

	img = map_sysmem(httpd_addr, httpd_size);
	type = detect_img_type(img, httpd_size);
	unmap_sysmem(img);

	set_img_env(httpd_addr, httpd_size, type);
	printf("httpd: received %lu bytes, type %s\n", httpd_size, type);

	if (!strcmp(type, "unknown")) {
		snprintf(msg, sizeof(msg),
			 "<div><p class=\"w\">%s</p></div>%s",
			 img_type_desc(type), page_form);
		httpd_respond(pcb, c, "200 OK", msg, false);
		return;
	}

	snprintf(msg, sizeof(msg),
		 "<div><p>Received %lu bytes.</p><p>%s</p>"
		 "<p>Follow the progress on the serial console or netconsole.</p></div>",
		 httpd_size, img_type_desc(type));
	httpd_respond(pcb, c, "200 OK", msg, true);
}

static bool hdr_get(const char *hdr, const char *name, char *val, int len)
{
	const char *p = hdr;
	int nlen = strlen(name);

	while ((p = strchr(p, '\n'))) {
		p++;
		if (!strncasecmp(p, name, nlen) && p[nlen] == ':') {
			int i = 0;

			p += nlen + 1;
			while (*p == ' ')
				p++;
			while (*p && *p != '\r' && *p != '\n' && i < len - 1)
				val[i++] = *p++;
			val[i] = '\0';
			return true;
		}
	}
	return false;
}

/* Returns the number of bytes of @data consumed by the request headers */
static int httpd_parse_headers(struct tcp_pcb *pcb, struct httpd_conn *c,
			       const u8 *data, int len)
{
	char val[128], clen[16];
	char *end, *b;
	int used = min(len, HTTPD_HDR_MAX - c->hdr_len);
	int hdr_bytes;

	memcpy(c->hdr + c->hdr_len, data, used);
	c->hdr_len += used;
	c->hdr[c->hdr_len] = '\0';

	end = strstr(c->hdr, "\r\n\r\n");
	if (!end) {
		if (c->hdr_len >= HTTPD_HDR_MAX)
			httpd_respond(pcb, c, "431 Request Header Fields Too Large",
				      "<div>Request too large.</div>", false);
		return len;
	}

	/* bytes of this segment that belong to the headers */
	hdr_bytes = (end + 4 - c->hdr) - (c->hdr_len - used);
	*end = '\0';
	c->hdr_done = true;

	if (!strncmp(c->hdr, "GET ", 4)) {
		httpd_respond(pcb, c, "200 OK", page_form, false);
		return len;
	}

	if (strncmp(c->hdr, "POST /upload", 12)) {
		httpd_respond(pcb, c, "404 Not Found",
			      "<div>Not found. <a href=\"/\">Back</a></div>",
			      false);
		return len;
	}

	if (!hdr_get(c->hdr, "Content-Length", clen, sizeof(clen)) ||
	    !hdr_get(c->hdr, "Content-Type", val, sizeof(val))) {
		httpd_respond(pcb, c, "411 Length Required",
			      "<div>Missing headers.</div>", false);
		return len;
	}
	c->content_len = simple_strtoul(clen, NULL, 10);
	b = strstr(val, "boundary=");
	if (!b || c->content_len == 0 || c->content_len > HTTPD_MAX_UPLOAD) {
		httpd_respond(pcb, c, "413 Payload Too Large",
			      "<div>Missing boundary or file too large.</div>",
			      false);
		return len;
	}
	b += 9;
	if (*b == '"')
		b++;
	strlcpy(c->boundary, b, sizeof(c->boundary));
	b = strchr(c->boundary, '"');
	if (b)
		*b = '\0';
	c->is_upload = true;
	printf("httpd: receiving %lu bytes\n", c->content_len);

	return hdr_bytes;
}

static err_t httpd_recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p,
			   err_t err)
{
	struct httpd_conn *c = arg;
	struct pbuf *q;

	if (!c) {
		if (p)
			pbuf_free(p);
		return ERR_OK;
	}
	if (!p) {
		/* peer closed */
		if (!c->tx_left)
			httpd_close(pcb, c);
		return ERR_OK;
	}

	for (q = p; q; q = q->next) {
		const u8 *data = q->payload;
		int len = q->len;

		if (c->close_when_sent)
			break;	/* response already queued: drain */

		if (!c->hdr_done) {
			int used = httpd_parse_headers(pcb, c, data, len);

			data += used;
			len -= used;
			if (!c->is_upload)
				continue;
		}

		if (c->is_upload && len > 0) {
			ulong n = min_t(ulong, len, c->content_len - c->received);
			void *dst = map_sysmem(httpd_addr + c->received, n);

			memcpy(dst, data, n);
			unmap_sysmem(dst);
			c->received += n;
			if (c->received == c->content_len ||
			    !(c->received % (1 << 20)))
				printf("\rhttpd: %lu / %lu KiB", c->received >> 10,
				       c->content_len >> 10);
			if (c->received == c->content_len) {
				printf("\n");
				httpd_upload_done(pcb, c);
			}
		}
	}

	tcp_recved(pcb, p->tot_len);
	pbuf_free(p);
	return ERR_OK;
}

static err_t httpd_accept_cb(void *arg, struct tcp_pcb *pcb, err_t err)
{
	struct httpd_conn *c;

	if (err != ERR_OK || !pcb)
		return ERR_VAL;

	c = calloc(1, sizeof(*c));
	if (!c)
		return ERR_MEM;

	tcp_arg(pcb, c);
	tcp_recv(pcb, httpd_recv_cb);
	tcp_sent(pcb, httpd_sent_cb);
	tcp_err(pcb, httpd_err_cb);
	return ERR_OK;
}

static int do_httpd(struct cmd_tbl *cmdtp, int flag, int argc,
		    char *const argv[])
{
	struct net_lwip_ctx net = {};
	struct tcp_pcb *pcb, *lpcb;
	int ret;

	httpd_addr = argc > 1 ? hextoul(argv[1], NULL) :
			       env_get_hex("loadaddr", CONFIG_SYS_LOAD_ADDR);
	httpd_size = 0;
	httpd_state = HTTPD_RUNNING;
	env_set("img_type", "none");

	ret = net_lwip_start(&net, NET_LWIP_ADDR_ENV_STRICT);
	if (ret) {
		printf("httpd: network start failed (%d)\n", ret);
		return CMD_RET_FAILURE;
	}

	pcb = tcp_new();
	if (!pcb || tcp_bind(pcb, IP_ANY_TYPE, HTTPD_PORT) != ERR_OK) {
		printf("httpd: cannot bind port %d\n", HTTPD_PORT);
		if (pcb)
			tcp_close(pcb);
		net_lwip_stop(&net);
		return CMD_RET_FAILURE;
	}
	lpcb = tcp_listen(pcb);
	tcp_accept(lpcb, httpd_accept_cb);

	printf("Web recovery: http://%s/ (Ctrl-C to stop)\n", env_get("ipaddr"));

	while (httpd_state == HTTPD_RUNNING) {
		net_lwip_poll();
		if (ctrlc()) {
			httpd_state = HTTPD_FAILED;
			break;
		}
	}

	/* let lwIP push out the last ACKs/FINs */
	if (httpd_state == HTTPD_UPLOADED) {
		ulong t = get_timer(0);

		while (get_timer(t) < 500)
			net_lwip_poll();
	}

	tcp_accept(lpcb, NULL);
	tcp_close(lpcb);
	net_lwip_stop(&net);

	return httpd_state == HTTPD_UPLOADED ? CMD_RET_SUCCESS :
					       CMD_RET_FAILURE;
}

U_BOOT_CMD(httpd, 2, 0, do_httpd,
	   "web recovery server: receive a firmware image over HTTP",
	   "[addr] - serve an upload page on port 80, store the image at addr\n"
	   "    (default $loadaddr); sets $filesize and $img_type"
);

static int do_imgtype(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	ulong addr, size;
	const char *type;
	u8 *p;

	if (argc < 3)
		return CMD_RET_USAGE;

	addr = hextoul(argv[1], NULL);
	size = hextoul(argv[2], NULL);
	p = map_sysmem(addr, size);
	type = detect_img_type(p, size);
	unmap_sysmem(p);

	set_img_env(addr, size, type);
	printf("image type: %s\n", type);
	return strcmp(type, "unknown") ? CMD_RET_SUCCESS : CMD_RET_FAILURE;
}

U_BOOT_CMD(imgtype, 3, 0, do_imgtype,
	   "detect firmware image type (ubi, uimage, kwb)",
	   "addr size - sets $img_type and $filesize_blk"
);
