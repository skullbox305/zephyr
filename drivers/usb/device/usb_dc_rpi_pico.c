/*
 * Copyright (c) 2021, Pete Johanson
 * Copyright (c) 2017 Christer Weinigel.
 * Copyright (c) 2017, I-SENSE group of ICCS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <string.h>
#include <usb/usb_device.h>
#include <sys/util.h>
#include <hardware/regs/usb.h>
#include <hardware/structs/usb.h>
#include <hardware/resets.h>

#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(udc_rpi, CONFIG_USB_DRIVER_LOG_LEVEL);

#define DT_DRV_COMPAT raspberrypi_pico_usbd

#define USB_BASE_ADDRESS DT_INST_REG_ADDR(0)
#define USB_IRQ DT_INST_IRQ_BY_NAME(0, usbctrl, irq)
#define USB_IRQ_PRI DT_INST_IRQ_BY_NAME(0, usbctrl, priority)
#define USB_NUM_BIDIR_ENDPOINTS DT_INST_PROP(0, num_bidir_endpoints)
/* Size of a USB SETUP packet */
#define SETUP_SIZE 8

/* Helper macros to make it easier to work with endpoint numbers */
#define EP0_IDX 0

#define DATA_BUFFER_SIZE 64U

/* Needed for pico-sdk */
#ifndef typeof
#define typeof __typeof__
#endif

#define usb_hw_set hw_set_alias(usb_hw)
#define usb_hw_clear hw_clear_alias(usb_hw)

/* Endpoint state */
struct udc_rpi_ep_state {
	uint16_t mps; /** Endpoint max packet size */
	enum usb_dc_ep_transfer_type type; /** Endpoint type */
	uint8_t halted; /** Endpoint stall flag */
	usb_dc_ep_callback cb; /** Endpoint callback function */
	uint32_t read_offset; /** Current offset in read buffer */
	struct k_sem write_sem; /** Write boolean semaphore */
	io_rw_32 *ep_ctl;
	io_rw_32 *buf_ctl;
	uint8_t *buf;
	uint8_t next_pid;
};

#define USBD_THREAD_STACK_SIZE		1024

/* Driver state */
struct udc_rpi_state {
	usb_dc_status_callback status_cb; /* Status callback */
	struct udc_rpi_ep_state out_ep_state[USB_NUM_BIDIR_ENDPOINTS];
	struct udc_rpi_ep_state in_ep_state[USB_NUM_BIDIR_ENDPOINTS];
	bool setup_available;
	bool should_set_address;
	uint8_t addr;

	K_KERNEL_STACK_MEMBER(thread_stack, USBD_THREAD_STACK_SIZE);
	struct k_thread thread;
};

static struct udc_rpi_state state;

#define USB_DC_CB_TYPE_MGMT		0
#define USB_DC_CB_TYPE_EP		1

struct cb_msg {
	uint8_t ep;
	uint8_t type;
	uint32_t cb;
};

K_MSGQ_DEFINE(usb_dc_msgq, sizeof(struct cb_msg), 10, 4);
static void udc_rpi_thread_main(void *arg1, void *unused1, void *unused2);

/* Internal functions */

static struct udc_rpi_ep_state *udc_rpi_get_ep_state(uint8_t ep)
{
	struct udc_rpi_ep_state *ep_state_base;

	if (USB_EP_GET_IDX(ep) >= USB_NUM_BIDIR_ENDPOINTS) {
		return NULL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		ep_state_base = state.out_ep_state;
	} else {
		ep_state_base = state.in_ep_state;
	}

	return ep_state_base + USB_EP_GET_IDX(ep);
}

static int udc_rpi_start_xfer(uint8_t ep, const void *data, size_t len)
{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);

	uint32_t val = len | USB_BUF_CTRL_AVAIL;

	if (USB_EP_DIR_IS_IN(ep)) {
		/* If writing to host, put data in place */

		if (data) {
			memcpy(ep_state->buf, data, len);
			val |= USB_BUF_CTRL_FULL;
		}
	} else {
		ep_state->read_offset = 0;

		/* TODO: this seems to be needed to get the setup stage working,
		 * otherwise I get DATA SEQ errors. This should be improved.
		 */

		if (USB_EP_GET_IDX(ep) == 0) {
			ep_state->next_pid = 1;
		}
	}

	LOG_DBG("xfer ep %d len %d pid: %d", ep, len, ep_state->next_pid);
	val |= ep_state->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;

	ep_state->next_pid ^= 1u;
	*ep_state->buf_ctl = val;

	return 0;
}

static void udc_rpi_handle_setup(void)
{
	struct cb_msg msg;

	LOG_DBG("");

	state.setup_available = true;

	/* Reset PID to 1 for EP0 IN */
	udc_rpi_get_ep_state(USB_CONTROL_EP_IN)->next_pid = 1;

	msg.ep = USB_CONTROL_EP_OUT;
	msg.cb = USB_DC_EP_SETUP;
	msg.type = USB_DC_CB_TYPE_EP;

	k_msgq_put(&usb_dc_msgq, &msg, K_NO_WAIT);
}

static void udc_rpi_handle_buff_status(void)
{
	struct udc_rpi_ep_state *ep;
	enum usb_dc_ep_cb_status_code status_code;
	uint8_t status = usb_hw->buf_status;
	unsigned int i;
	unsigned int bit = 1U;
	struct cb_msg msg;

	LOG_DBG("status: %d", status);

	for (i = 0U; status && i < USB_NUM_BIDIR_ENDPOINTS * 2; i++) {
		if (status & bit) {
			usb_hw_clear->buf_status = bit;
			bool in = !(i & 1U);
			uint8_t ep_addr = (i >> 1U) | (in ? USB_EP_DIR_IN : USB_EP_DIR_OUT);

			ep = udc_rpi_get_ep_state(ep_addr);
			status_code = in ? USB_DC_EP_DATA_IN : USB_DC_EP_DATA_OUT;

			LOG_DBG("buff ep %i in? %i", (i >> 1), in);

			if (i == 0 && in && state.should_set_address) {
				state.should_set_address = false;
				usb_hw->dev_addr_ctrl = state.addr;
			}

			if (in) {
				k_sem_give(&ep->write_sem);
			}

			msg.ep = ep_addr;
			msg.type = USB_DC_CB_TYPE_EP;
			msg.cb = status_code;

			k_msgq_put(&usb_dc_msgq, &msg, K_NO_WAIT);

			status &= ~bit;
		}

		bit <<= 1U;
	}
}

static void udc_rpi_isr(const void *arg)
{
	uint32_t status = usb_hw->ints;
	uint32_t handled = 0;
	struct cb_msg msg;

	if (status & USB_INTS_SETUP_REQ_BITS) {
		handled |= USB_INTS_SETUP_REQ_BITS;
		usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
		udc_rpi_handle_setup();
	}

	if (status & USB_INTS_BUFF_STATUS_BITS) {
		handled |= USB_INTS_BUFF_STATUS_BITS;
		udc_rpi_handle_buff_status();
	}

	if (status & USB_INTS_DEV_CONN_DIS_BITS) {
		LOG_DBG("buf %u ep %u", *udc_rpi_get_ep_state(0x81)->buf_ctl,
			*udc_rpi_get_ep_state(0x81)->ep_ctl);
		handled |= USB_INTS_DEV_CONN_DIS_BITS;
		usb_hw_clear->sie_status = USB_SIE_STATUS_CONNECTED_BITS;

		msg.ep = 0U;
		msg.cb = usb_hw->sie_status & USB_SIE_STATUS_CONNECTED_BITS ?
			USB_DC_CONNECTED :
			USB_DC_DISCONNECTED;
		msg.type = 0U;

		k_msgq_put(&usb_dc_msgq, &msg, K_NO_WAIT);
	}

	if (status & USB_INTS_BUS_RESET_BITS) {
		int i;

		LOG_WRN("BUS RESET");
		handled |= USB_INTS_BUS_RESET_BITS;
		usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
		usb_hw->dev_addr_ctrl = 0;

		/* The DataInCallback will never be called at this point for any pending
		 * transactions. Reset the IN semaphores to prevent perpetual locked state.
		 */

		for (i = 0; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
			k_sem_give(&state.in_ep_state[i].write_sem);
		}

		msg.ep = 0U;
		msg.cb = USB_DC_RESET;
		msg.type = 0U;

		k_msgq_put(&usb_dc_msgq, &msg, K_NO_WAIT);
	}

	if (status & USB_INTS_ERROR_DATA_SEQ_BITS) {
		LOG_WRN("data seq");
		usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;
		handled |= USB_INTS_ERROR_DATA_SEQ_BITS;
	}

	if (status ^ handled) {
		LOG_ERR("unhandled IRQ: 0x%x", (uint)(status ^ handled));
	}
}

void udc_rpi_init_bidir_endpoint(uint8_t i)
{
	state.out_ep_state[i].buf_ctl = &usb_dpram->ep_buf_ctrl[i].out;
	state.in_ep_state[i].buf_ctl = &usb_dpram->ep_buf_ctrl[i].in;

	if (i != EP0_IDX) {
		state.out_ep_state[i].ep_ctl = &usb_dpram->ep_ctrl[i - 1].out;
		state.in_ep_state[i].ep_ctl = &usb_dpram->ep_ctrl[i - 1].in;

		state.out_ep_state[i].buf =
			&usb_dpram->epx_data[((i - 1) * 2 + 1) * DATA_BUFFER_SIZE];
		state.in_ep_state[i].buf = &usb_dpram->epx_data[((i - 1) * 2) * DATA_BUFFER_SIZE];
	} else {
		state.out_ep_state[i].buf = &usb_dpram->ep0_buf_a[0];
		state.in_ep_state[i].buf = &usb_dpram->ep0_buf_a[0];
	}

	k_sem_init(&state.in_ep_state[i].write_sem, 1, 1);
}

static int udc_rpi_init(void)
{
	unsigned int i;

	/* Reset usb controller */
	reset_block(RESETS_RESET_USBCTRL_BITS);
	unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

	/* Clear any previous state in dpram/hw just in case */
	memset(usb_hw, 0, sizeof(*usb_hw));
	memset(usb_dpram, 0, sizeof(*usb_dpram));

	/* Mux the controller to the onboard usb phy */
	usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;

	/* Force VBUS detect so the device thinks it is plugged into a host */
	usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;

	/* Enable the USB controller in device mode. */
	usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

	/* Enable an interrupt per EP0 transaction */
	usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;

	/* Enable interrupts for when a buffer is done, when the bus is reset,
	 * and when a setup packet is received, and device connection status
	 */
	usb_hw->inte = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS |
		       USB_INTS_DEV_CONN_DIS_BITS |
		       USB_INTS_SETUP_REQ_BITS | /*USB_INTS_EP_STALL_NAK_BITS |*/
		       USB_INTS_ERROR_BIT_STUFF_BITS | USB_INTS_ERROR_CRC_BITS |
		       USB_INTS_ERROR_DATA_SEQ_BITS | USB_INTS_ERROR_RX_OVERFLOW_BITS |
		       USB_INTS_ERROR_RX_TIMEOUT_BITS;

	/* Set up endpoints (endpoint control registers)
	 * described by device configuration
	 * usb_setup_endpoints();
	 */
	for (i = 0U; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
		udc_rpi_init_bidir_endpoint(i);
	}

	k_thread_create(&state.thread, state.thread_stack,
			USBD_THREAD_STACK_SIZE,
			udc_rpi_thread_main, NULL, NULL, NULL,
			K_PRIO_COOP(2), 0, K_NO_WAIT);

	IRQ_CONNECT(USB_IRQ, USB_IRQ_PRI, udc_rpi_isr, 0, 0);
	irq_enable(USB_IRQ);

	/* Present full speed device by enabling pull up on DP */
	usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;

	return 0;
}

/* Zephyr USB device controller API implementation */

int usb_dc_attach(void)
{
	int ret;

	LOG_DBG("");

	ret = udc_rpi_init();
	if (ret) {
		return ret;
	}

	return 0;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	ep_state->cb = cb;

	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	LOG_DBG("");

	state.status_cb = cb;
}

int usb_dc_set_address(const uint8_t addr)
{
	LOG_DBG("addr %u (0x%02x)", addr, addr);

	state.should_set_address = true;
	state.addr = addr;

	return 0;
}

int usb_dc_ep_start_read(uint8_t ep, size_t len)
{
	int ret;

	LOG_DBG("ep 0x%02x len %d", ep, len);

	/* we flush USB_CONTROL_EP_IN by doing a 0 length receive on it */
	if (!USB_EP_DIR_IS_OUT(ep) && (ep != USB_CONTROL_EP_IN || len)) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	if (len > DATA_BUFFER_SIZE) {
		len = DATA_BUFFER_SIZE;
	}

	ret = udc_rpi_start_xfer(ep, NULL, len);

	return ret;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data *const cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);

	LOG_DBG("ep %x, mps %d, type %d", cfg->ep_addr, cfg->ep_mps, cfg->ep_type);

	if ((cfg->ep_type == USB_DC_EP_CONTROL) && ep_idx) {
		LOG_ERR("invalid endpoint configuration");
		return -1;
	}

	if (ep_idx > (USB_NUM_BIDIR_ENDPOINTS - 1)) {
		LOG_ERR("endpoint index/address out of range");
		return -1;
	}

	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const ep_cfg)
{
	uint8_t ep = ep_cfg->ep_addr;
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("ep 0x%02x, previous mps %u, mps %u, type %u", ep_cfg->ep_addr, ep_state->mps,
		ep_cfg->ep_mps, ep_cfg->ep_type);

	ep_state->mps = ep_cfg->ep_mps;
	ep_state->type = ep_cfg->ep_type;

	return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}
	if (USB_EP_GET_IDX(ep) == 0) {
		usb_hw_set->ep_stall_arm = USB_EP_DIR_IS_OUT(ep) ?
			USB_EP_STALL_ARM_EP0_OUT_BITS :
			USB_EP_STALL_ARM_EP0_IN_BITS;
	}

	*ep_state->buf_ctl = USB_BUF_CTRL_STALL;

	ep_state->halted = 1U;

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);
	uint8_t val;

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	if (USB_EP_GET_IDX(ep) > 0) {
		val = *ep_state->buf_ctl;
		val &= ~USB_BUF_CTRL_STALL;

		*ep_state->buf_ctl = val;

		ep_state->halted = 0U;
		ep_state->read_offset = 0U;
	}

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state || !stalled) {
		return -EINVAL;
	}

	*stalled = ep_state->halted;

	return 0;
}

static inline uint32_t usb_dc_ep_rpi_pico_buffer_offset(volatile uint8_t *buf)
{
	/* TODO: Bits 0-5 are ignored by the controller so make sure these are 0 */
	return (uint32_t)buf ^ (uint32_t)usb_dpram;
}

int usb_dc_ep_enable(const uint8_t ep)
{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("ep 0x%02x (id: %d) -> type %d", ep, USB_EP_GET_IDX(ep), ep_state->type);

	/* clear buffer state (EP0 starts with PID=1 for setup phase) */

	*ep_state->buf_ctl = (USB_EP_GET_IDX(ep) == 0 ? USB_BUF_CTRL_DATA1_PID : 0);

	/* EP0 doesn't have an ep_ctl */
	if (ep_state->ep_ctl) {
		uint32_t val =
			EP_CTRL_ENABLE_BITS |
			EP_CTRL_INTERRUPT_PER_BUFFER |
			(ep_state->type << EP_CTRL_BUFFER_TYPE_LSB) |
			usb_dc_ep_rpi_pico_buffer_offset(ep_state->buf);

		*ep_state->ep_ctl = val;
	}

	if (USB_EP_DIR_IS_OUT(ep) && ep != USB_CONTROL_EP_OUT) {
		return usb_dc_ep_start_read(ep, DATA_BUFFER_SIZE);
	}

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	/* EP0 doesn't have an ep_ctl */
	if (!ep_state->ep_ctl) {
		return 0;
	}

	uint8_t val = *ep_state->ep_ctl & ~EP_CTRL_ENABLE_BITS;

	*ep_state->ep_ctl = val;

	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data, const uint32_t data_len,
		    uint32_t *const ret_bytes)
{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);
	uint32_t len = data_len;
	int ret = 0;

	LOG_DBG("ep 0x%02x, len %u", ep, data_len);

	if (!ep_state || !USB_EP_DIR_IS_IN(ep)) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	if (ep == USB_CONTROL_EP_IN && len > USB_MAX_CTRL_MPS) {
		len = USB_MAX_CTRL_MPS;
	} else if (len > ep_state->mps) {
		len = ep_state->mps;
	}

	ret = k_sem_take(&ep_state->write_sem, K_NO_WAIT);
	if (ret) {
		LOG_ERR("Unable to get write lock (%d)", ret);
		return -EAGAIN;
	}

	if (!k_is_in_isr()) {
		irq_disable(USB_IRQ);
	}

	ret = udc_rpi_start_xfer(ep, data, len);

	if (ret < 0) {
		LOG_ERR("xfer failed (%d)", ret);
		k_sem_give(&ep_state->write_sem);
		ret = -EIO;
	}

	if (!k_is_in_isr()) {
		irq_enable(USB_IRQ);
	}

	if (ret >= 0 && ret_bytes != NULL) {
		*ret_bytes = len;
	}

	return ret;
}

uint32_t udc_rpi_get_ep_buffer_len(const uint8_t ep)
{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);
	uint32_t buf_ctl = *ep_state->buf_ctl;

	return buf_ctl & USB_BUF_CTRL_LEN_MASK;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len, uint32_t *read_bytes)
{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);
	uint32_t read_count;

	if (!ep_state) {
		LOG_ERR("Invalid Endpoint %x", ep);
		return -EINVAL;
	}

	if (!USB_EP_DIR_IS_OUT(ep)) {
		LOG_ERR("Wrong endpoint direction: 0x%02x", ep);
		return -EINVAL;
	}

	if (state.setup_available) {
		read_count = SETUP_SIZE;
	} else {
		read_count = udc_rpi_get_ep_buffer_len(ep) - ep_state->read_offset;
	}

	LOG_DBG("ep 0x%02x, %u bytes, %u+%u, %p", ep, max_data_len, ep_state->read_offset,
		read_count, data);

	if (data) {
		read_count = MIN(read_count, max_data_len);

		if (state.setup_available) {
			memcpy(data, (const void *)&usb_dpram->setup_packet, read_count);
		} else {
			memcpy(data, ep_state->buf + ep_state->read_offset, read_count);
		}

		ep_state->read_offset += read_count;
	} else if (max_data_len) {
		LOG_ERR("Wrong arguments");
	}

	if (read_bytes) {
		*read_bytes = read_count;
	}

	return 0;
}

int usb_dc_ep_read_continue(uint8_t ep)

{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);

	if (!ep_state || !USB_EP_DIR_IS_OUT(ep)) {
		LOG_ERR("Not valid endpoint: %02x", ep);
		return -EINVAL;
	}

	size_t bytes_received =
		(state.setup_available ? SETUP_SIZE : udc_rpi_get_ep_buffer_len(ep));

	state.setup_available = false;

	/* If no more data in the buffer, start a new read transaction. */
	LOG_DBG("received %d offset: %d", bytes_received, ep_state->read_offset);
	if (bytes_received == ep_state->read_offset) {
		return usb_dc_ep_start_read(ep, DATA_BUFFER_SIZE);
	}

	return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t *const read_bytes)
{
	if (usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes) != 0) {
		return -EINVAL;
	}

	if (!max_data_len) {
		return 0;
	}

	if (usb_dc_ep_read_continue(ep) != 0) {
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	return usb_dc_ep_set_stall(ep);
}

int usb_dc_ep_flush(const uint8_t ep)
{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_ERR("Not implemented");

	return 0;
}

int usb_dc_ep_mps(const uint8_t ep)
{
	struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	return ep_state->mps;
}

int usb_dc_detach(void)
{
	LOG_ERR("Not implemented");

	return 0;
}

int usb_dc_reset(void)
{
	LOG_ERR("Not implemented");

	return 0;
}

/*
 * This thread is only used to not run the USB device stack and endpoint
 * callbacks in the ISR context, which happens when an callback function
 * is called. TODO: something similar should be implemented in the USB
 * device stack so that it can be used by all drivers.
 */
static void udc_rpi_thread_main(void *arg1, void *unused1, void *unused2)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	struct cb_msg msg;

	while (true) {
		k_msgq_get(&usb_dc_msgq, &msg, K_FOREVER);

		if (msg.type == USB_DC_CB_TYPE_EP) {
			struct udc_rpi_ep_state *ep_state = udc_rpi_get_ep_state(msg.ep);

			switch (msg.cb) {
			case USB_DC_EP_SETUP:
				if (ep_state->cb) {
					ep_state->cb(msg.ep,
						USB_DC_EP_SETUP);
				}
				break;
			case USB_DC_EP_DATA_OUT:
				if (ep_state->cb) {
					ep_state->cb(msg.ep,
						USB_DC_EP_DATA_OUT);
				}
				break;
			case USB_DC_EP_DATA_IN:
				if (ep_state->cb) {
					ep_state->cb(msg.ep,
						USB_DC_EP_DATA_IN);
				}
				break;
			default:
				LOG_ERR("unknown msg");
				break;
			}
		} else if (state.status_cb) {
			switch (msg.cb) {
			case USB_DC_RESET:
				state.status_cb(USB_DC_RESET, NULL);
				break;
			case USB_DC_ERROR:
				state.status_cb(USB_DC_ERROR, NULL);
				break;
			case USB_DC_SUSPEND:
				state.status_cb(USB_DC_SUSPEND, NULL);
				break;
			case USB_DC_RESUME:
				state.status_cb(USB_DC_RESUME, NULL);
				break;
			default:
				LOG_ERR("unknown msg");
				break;
			}
		}
	}
}
