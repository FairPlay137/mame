// license:BSD-3-Clause
// copyright-holders:FairPlay137,wtvemac

/***********************************************************************************************

    spot_asic.cpp

    WebTV Networks Inc. SPOT ASIC

    This ASIC controls most of the I/O on the 1st generation WebTV hardware. It is also referred
    to as FIDO on the actual chip that implements the SPOT logic.

    This implementation is based off of both the archived technical specifications, as well as
    the various reverse-engineering efforts of the WebTV community.

    The SPOT ASIC is split into multiple "units", just like its successor.

    The romUnit (0xA4001xxx) provides a shared interface to the ROM.

    The audUnit (0xA4002xxx) handles audio DMA.

    The vidUnit (0xA4003xxx) handles video DMA.

    The devUnit (0xA4004xxx) handles GPIO, IR input, front panel LEDs, modem interfacing, and the
    PS/2 keyboard port.

    The memUnit (0xA4005xxx) handles memory timing and other memory-related operations. These
    registers are only emulated for completeness; they do not have an effect on the emulation.

****************************************************************************************************/
#include "emu.h"

#include "machine/input_merger.h"
#include "render.h"
#include "spot_asic.h"
#include "screen.h"
#include "main.h"
#include "machine.h"
#include "config.h"

#define LOG_UNKNOWN     (1U << 1)
#define LOG_READS       (1U << 2)
#define LOG_WRITES      (1U << 3)
#define LOG_ERRORS      (1U << 4)
#define LOG_I2C_IGNORES (1U << 5)
#define LOG_DEFAULT     (LOG_READS | LOG_WRITES | LOG_ERRORS | LOG_I2C_IGNORES | LOG_UNKNOWN)

#define VERBOSE         (LOG_DEFAULT)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(SPOT_ASIC, spot_asic_device, "spot_asic", "WebTV SPOT ASIC")

spot_asic_device::spot_asic_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SPOT_ASIC, tag, owner, clock),
	device_serial_interface(mconfig, *this),
	device_video_interface(mconfig, *this),
	m_hostcpu(*this, finder_base::DUMMY_TAG),
	m_serial_id(*this, finder_base::DUMMY_TAG),
	m_nvram(*this, finder_base::DUMMY_TAG),
	m_kbdc(*this, "kbdc"),
	m_screen(*this, "screen"),
	m_dac(*this, "dac%u", 0),
	m_lspeaker(*this, "lspeaker"),
	m_rspeaker(*this, "rspeaker"),
    m_modem_uart(*this, "modem_uart"),
	m_watchdog(*this, "watchdog"),
    m_sys_config(*owner, "sys_config"),
    m_emu_config(*owner, "emu_config"),
    m_power_led(*this, "power_led"),
    m_connect_led(*this, "connect_led"),
    m_message_led(*this, "message_led")
{
}

static DEVICE_INPUT_DEFAULTS_START( wtv_modem )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_115200 )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_115200 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_NONE )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
DEVICE_INPUT_DEFAULTS_END

DECLARE_INPUT_CHANGED_MEMBER(pbuff_index_changed);

void spot_asic_device::bus_unit_map(address_map &map)
{
	map(0x000, 0x003).r(FUNC(spot_asic_device::reg_0000_r));                                      // BUS_CHIPID
	map(0x004, 0x007).rw(FUNC(spot_asic_device::reg_0004_r), FUNC(spot_asic_device::reg_0004_w)); // BUS_CHIPCNTL
	map(0x008, 0x00b).r(FUNC(spot_asic_device::reg_0008_r));                                      // BUS_INTSTAT
	map(0x108, 0x10b).w(FUNC(spot_asic_device::reg_0108_w));                                      // BUS_INTEN_S
	map(0x00c, 0x00f).rw(FUNC(spot_asic_device::reg_000c_r), FUNC(spot_asic_device::reg_000c_w)); // BUS_ERRSTAT
	map(0x10c, 0x10f).w(FUNC(spot_asic_device::reg_010c_w));                                      // BUS_INTEN_C
	map(0x010, 0x013).r(FUNC(spot_asic_device::reg_0010_r));                                      // BUS_ERRSTAT
	map(0x110, 0x113).w(FUNC(spot_asic_device::reg_0110_w));                                      // BUS_ERRSTAT_C
	map(0x014, 0x017).rw(FUNC(spot_asic_device::reg_0014_r), FUNC(spot_asic_device::reg_0014_w)); // BUS_ERREN_S
	map(0x114, 0x117).w(FUNC(spot_asic_device::reg_0114_w));                                      // BUS_ERREN_C
	map(0x018, 0x01b).r(FUNC(spot_asic_device::reg_0018_r));                                      // BUS_ERRADDR
	map(0x118, 0x11b).w(FUNC(spot_asic_device::reg_0118_w));                                      // BUS_WDREG_C
	map(0x01c, 0x01f).rw(FUNC(spot_asic_device::reg_001c_r), FUNC(spot_asic_device::reg_001c_w)); // BUS_FENADDR1
	map(0x020, 0x023).rw(FUNC(spot_asic_device::reg_0020_r), FUNC(spot_asic_device::reg_0020_w)); // BUS_FENMASK1
	map(0x024, 0x027).rw(FUNC(spot_asic_device::reg_0024_r), FUNC(spot_asic_device::reg_0024_w)); // BUS_FENADDR2
	map(0x028, 0x02b).rw(FUNC(spot_asic_device::reg_0028_r), FUNC(spot_asic_device::reg_0028_w)); // BUS_FENMASK2
}

void spot_asic_device::rom_unit_map(address_map &map)
{
	map(0x000, 0x003).r(FUNC(spot_asic_device::reg_1000_r));                                      // ROM_SYSCONFIG
	map(0x004, 0x007).rw(FUNC(spot_asic_device::reg_1004_r), FUNC(spot_asic_device::reg_1004_w)); // ROM_CNTL0
	map(0x008, 0x00b).rw(FUNC(spot_asic_device::reg_1008_r), FUNC(spot_asic_device::reg_1008_w)); // ROM_CNTL1
}

void spot_asic_device::aud_unit_map(address_map &map)
{
	map(0x000, 0x003).r(FUNC(spot_asic_device::reg_2000_r));                                      // AUD_CSTART
	map(0x004, 0x007).r(FUNC(spot_asic_device::reg_2004_r));                                      // AUD_CSIZE
	map(0x008, 0x00b).rw(FUNC(spot_asic_device::reg_2008_r), FUNC(spot_asic_device::reg_2008_w)); // AUD_CCONFIG
	map(0x00c, 0x00f).r(FUNC(spot_asic_device::reg_200c_r));                                      // AUD_CCNT
	map(0x010, 0x013).rw(FUNC(spot_asic_device::reg_2010_r), FUNC(spot_asic_device::reg_2010_w)); // AUD_NSTART
	map(0x014, 0x017).rw(FUNC(spot_asic_device::reg_2014_r), FUNC(spot_asic_device::reg_2014_w)); // AUD_NSIZE
	map(0x018, 0x01b).rw(FUNC(spot_asic_device::reg_2018_r), FUNC(spot_asic_device::reg_2018_w)); // AUD_NCONFIG
	map(0x01c, 0x01f).rw(FUNC(spot_asic_device::reg_201c_r), FUNC(spot_asic_device::reg_201c_w)); // AUD_DMACNTL
}

void spot_asic_device::vid_unit_map(address_map &map)
{
	map(0x000, 0x003).r(FUNC(spot_asic_device::reg_3000_r));                                      // VID_CSTART
	map(0x004, 0x007).r(FUNC(spot_asic_device::reg_3004_r));                                      // VID_CSIZE
	map(0x008, 0x00b).r(FUNC(spot_asic_device::reg_3008_r));                                      // VID_CCNT
	map(0x00c, 0x00f).rw(FUNC(spot_asic_device::reg_300c_r), FUNC(spot_asic_device::reg_300c_w)); // VID_NSTART
	map(0x010, 0x013).rw(FUNC(spot_asic_device::reg_3010_r), FUNC(spot_asic_device::reg_3010_w)); // VID_NSIZE
	map(0x014, 0x017).rw(FUNC(spot_asic_device::reg_3014_r), FUNC(spot_asic_device::reg_3014_w)); // VID_DMACNTL
	map(0x018, 0x01b).rw(FUNC(spot_asic_device::reg_3018_r), FUNC(spot_asic_device::reg_3018_w)); // VID_FCNTL
	map(0x01c, 0x01f).rw(FUNC(spot_asic_device::reg_301c_r), FUNC(spot_asic_device::reg_301c_w)); // VID_BLNKCOL
	map(0x020, 0x023).rw(FUNC(spot_asic_device::reg_3020_r), FUNC(spot_asic_device::reg_3020_w)); // VID_HSTART
	map(0x024, 0x027).rw(FUNC(spot_asic_device::reg_3024_r), FUNC(spot_asic_device::reg_3024_w)); // VID_HSIZE
	map(0x028, 0x02b).rw(FUNC(spot_asic_device::reg_3028_r), FUNC(spot_asic_device::reg_3028_w)); // VID_VSTART
	map(0x02c, 0x02f).rw(FUNC(spot_asic_device::reg_302c_r), FUNC(spot_asic_device::reg_302c_w)); // VID_VSIZE
	map(0x030, 0x033).rw(FUNC(spot_asic_device::reg_3030_r), FUNC(spot_asic_device::reg_3030_w)); // VID_HINTLINE
	map(0x034, 0x037).r(FUNC(spot_asic_device::reg_3034_r));                                      // VID_CLINE
	map(0x038, 0x03b).r(FUNC(spot_asic_device::reg_3038_r));                                      // VID_INTSTAT
	map(0x138, 0x13b).w(FUNC(spot_asic_device::reg_3138_w));                                      // VID_INTSTAT_C
	map(0x03c, 0x03f).rw(FUNC(spot_asic_device::reg_303c_r), FUNC(spot_asic_device::reg_303c_w)); // VID_INTEN_S
	map(0x13c, 0x13f).w(FUNC(spot_asic_device::reg_313c_w));                                      // VID_INTEN_C
}

void spot_asic_device::dev_unit_map(address_map &map)
{
	map(0x000, 0x003).r(FUNC(spot_asic_device::reg_4000_r));                                      // DEV_IRDATA
	map(0x004, 0x007).rw(FUNC(spot_asic_device::reg_4004_r), FUNC(spot_asic_device::reg_4004_w)); // DEV_LED
	map(0x008, 0x00b).rw(FUNC(spot_asic_device::reg_4008_r), FUNC(spot_asic_device::reg_4008_w)); // DEV_IDCNTL
	map(0x00c, 0x00f).rw(FUNC(spot_asic_device::reg_400c_r), FUNC(spot_asic_device::reg_400c_w)); // DEV_NVCNTL
	map(0x010, 0x013).rw(FUNC(spot_asic_device::reg_4010_r), FUNC(spot_asic_device::reg_4010_w)); // DEV_SCCNTL
	map(0x014, 0x017).rw(FUNC(spot_asic_device::reg_4014_r), FUNC(spot_asic_device::reg_4014_w)); // DEV_EXTTIME
	map(0x020, 0x023).rw(FUNC(spot_asic_device::reg_4020_r), FUNC(spot_asic_device::reg_4020_w)); // DEV_KBD0
	map(0x024, 0x027).rw(FUNC(spot_asic_device::reg_4024_r), FUNC(spot_asic_device::reg_4024_w)); // DEV_KBD1
	map(0x028, 0x02b).rw(FUNC(spot_asic_device::reg_4028_r), FUNC(spot_asic_device::reg_4028_w)); // DEV_KBD2
	map(0x02c, 0x02f).rw(FUNC(spot_asic_device::reg_402c_r), FUNC(spot_asic_device::reg_402c_w)); // DEV_KBD3
	map(0x030, 0x033).rw(FUNC(spot_asic_device::reg_4030_r), FUNC(spot_asic_device::reg_4030_w)); // DEV_KBD4
	map(0x034, 0x037).rw(FUNC(spot_asic_device::reg_4034_r), FUNC(spot_asic_device::reg_4034_w)); // DEV_KBD5
	map(0x038, 0x03b).rw(FUNC(spot_asic_device::reg_4038_r), FUNC(spot_asic_device::reg_4038_w)); // DEV_KBD6
	map(0x03c, 0x03f).rw(FUNC(spot_asic_device::reg_403c_r), FUNC(spot_asic_device::reg_403c_w)); // DEV_KBD7
	map(0x040, 0x043).rw(FUNC(spot_asic_device::reg_4040_r), FUNC(spot_asic_device::reg_4040_w)); // DEV_MOD0
	map(0x044, 0x047).rw(FUNC(spot_asic_device::reg_4044_r), FUNC(spot_asic_device::reg_4044_w)); // DEV_MOD1
	map(0x048, 0x04b).rw(FUNC(spot_asic_device::reg_4048_r), FUNC(spot_asic_device::reg_4048_w)); // DEV_MOD2
	map(0x04c, 0x04f).rw(FUNC(spot_asic_device::reg_404c_r), FUNC(spot_asic_device::reg_404c_w)); // DEV_MOD3
	map(0x050, 0x053).rw(FUNC(spot_asic_device::reg_4050_r), FUNC(spot_asic_device::reg_4050_w)); // DEV_MOD4
	map(0x054, 0x057).rw(FUNC(spot_asic_device::reg_4054_r), FUNC(spot_asic_device::reg_4054_w)); // DEV_MOD5
	map(0x058, 0x05b).rw(FUNC(spot_asic_device::reg_4058_r), FUNC(spot_asic_device::reg_4058_w)); // DEV_MOD6
	map(0x05c, 0x05f).rw(FUNC(spot_asic_device::reg_405c_r), FUNC(spot_asic_device::reg_405c_w)); // DEV_MOD7
}

void spot_asic_device::mem_unit_map(address_map &map)
{
	map(0x000, 0x003).rw(FUNC(spot_asic_device::reg_5000_r), FUNC(spot_asic_device::reg_5000_w)); // MEM_CNTL
	map(0x004, 0x007).rw(FUNC(spot_asic_device::reg_5004_r), FUNC(spot_asic_device::reg_5004_w)); // MEM_REFCNT
	map(0x008, 0x00b).rw(FUNC(spot_asic_device::reg_5008_r), FUNC(spot_asic_device::reg_5008_w)); // MEM_DATA
	map(0x00c, 0x00f).w(FUNC(spot_asic_device::reg_500c_w));                                      // MEM_CMD
	map(0x010, 0x013).rw(FUNC(spot_asic_device::reg_5010_r), FUNC(spot_asic_device::reg_5010_w)); // MEM_TIMING
}

void spot_asic_device::device_add_mconfig(machine_config &config)
{
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_screen_update(FUNC(spot_asic_device::screen_update));
	m_screen->screen_vblank().set(FUNC(spot_asic_device::vblank_irq));
	m_screen->set_raw(VID_DEFAULT_XTAL, VID_DEFAULT_HTOTAL, 0, VID_DEFAULT_HBSTART, VID_DEFAULT_VTOTAL, 0, VID_DEFAULT_VBSTART);

	SPEAKER(config, m_lspeaker).front_left();
	SPEAKER(config, m_rspeaker).front_right();
	DAC_16BIT_R2R_TWOS_COMPLEMENT(config, m_dac[0], 0).add_route(0, m_lspeaker, 0.0);
	DAC_16BIT_R2R_TWOS_COMPLEMENT(config, m_dac[1], 0).add_route(0, m_rspeaker, 0.0);

	NS16550(config, m_modem_uart, 1.8432_MHz_XTAL);
	m_modem_uart->out_tx_callback().set("modem", FUNC(rs232_port_device::write_txd));
	m_modem_uart->out_dtr_callback().set("modem", FUNC(rs232_port_device::write_dtr));
	m_modem_uart->out_rts_callback().set("modem", FUNC(rs232_port_device::write_rts));
	m_modem_uart->out_int_callback().set(FUNC(spot_asic_device::irq_modem_w));

	rs232_port_device &rs232(RS232_PORT(config, "modem", default_rs232_devices, "null_modem"));
	rs232.set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(wtv_modem));
	rs232.rxd_handler().set(m_modem_uart, FUNC(ns16450_device::rx_w));
	rs232.dcd_handler().set(m_modem_uart, FUNC(ns16450_device::dcd_w));
	rs232.dsr_handler().set(m_modem_uart, FUNC(ns16450_device::dsr_w));
	rs232.ri_handler().set(m_modem_uart, FUNC(ns16450_device::ri_w));
	rs232.cts_handler().set(m_modem_uart, FUNC(ns16450_device::cts_w));

	KBDC8042(config, m_kbdc);
	m_kbdc->set_keyboard_type(kbdc8042_device::KBDC8042_PS2);
	m_kbdc->input_buffer_full_callback().set(FUNC(spot_asic_device::irq_keyboard_w));
	m_kbdc->system_reset_callback().set_inputline(":maincpu", INPUT_LINE_RESET);
	m_kbdc->set_keyboard_tag("at_keyboard");

	at_keyboard_device &at_keyb(AT_KEYB(config, "at_keyboard", pc_keyboard_device::KEYBOARD_TYPE::AT, 1));
	at_keyb.keypress().set(m_kbdc, FUNC(kbdc8042_device::keyboard_w));

	WATCHDOG_TIMER(config, m_watchdog);
	spot_asic_device::watchdog_enable(0);
}

void spot_asic_device::device_resolve_objects()
{
	// This grabs the configuration before it's usually done so we can read the PAL or NTSC bit to configure the screen.
	machine().manager().before_load_settings(machine());
	machine().configuration().load_settings();

	if (m_sys_config->read() & SYSCONFIG_NTSC)
		m_screen->set_raw(NTSC_SCREEN_XTAL, NTSC_SCREEN_HTOTAL, 0, NTSC_SCREEN_HBSTART, NTSC_SCREEN_VTOTAL, 0, NTSC_SCREEN_VBSTART);
	else
		m_screen->set_raw(PAL_SCREEN_XTAL, PAL_SCREEN_HTOTAL, 0, PAL_SCREEN_HBSTART, PAL_SCREEN_VTOTAL, 0, PAL_SCREEN_VBSTART);
}

void spot_asic_device::device_start()
{
	m_power_led.resolve();
	m_connect_led.resolve();
	m_message_led.resolve();

	dac_update_timer = timer_alloc(FUNC(spot_asic_device::dac_update), this);
	modem_buffer_timer = timer_alloc(FUNC(spot_asic_device::flush_modem_buffer), this);

	spot_asic_device::device_reset();

	save_item(NAME(m_intenable));
	save_item(NAME(m_intstat));
	save_item(NAME(m_errenable));
	save_item(NAME(m_chpcntl));
	save_item(NAME(m_wdenable));
	save_item(NAME(m_errstat));
	save_item(NAME(m_vid_nstart));
	save_item(NAME(m_vid_nsize));
	save_item(NAME(m_vid_dmacntl));
	save_item(NAME(m_vid_hstart));
	save_item(NAME(m_vid_hsize));
	save_item(NAME(m_vid_vstart));
	save_item(NAME(m_vid_vsize));
	save_item(NAME(m_vid_fcntl));
	save_item(NAME(m_vid_blank_color));
	save_item(NAME(m_vid_hintline));
	save_item(NAME(m_vid_cstart));
	save_item(NAME(m_vid_csize));
	save_item(NAME(m_vid_ccnt));
	save_item(NAME(m_vid_cline));
	save_item(NAME(m_vid_draw_nstart));
	save_item(NAME(m_vid_draw_hstart));
	save_item(NAME(m_vid_draw_hsize));
	save_item(NAME(m_vid_draw_vstart));
	save_item(NAME(m_vid_draw_vsize));
	save_item(NAME(m_vid_draw_blank_color));

	save_item(NAME(m_aud_cstart));
	save_item(NAME(m_aud_csize));
	save_item(NAME(m_aud_cconfig));
	save_item(NAME(m_aud_ccnt));
	save_item(NAME(m_aud_nstart));
	save_item(NAME(m_aud_nsize));
	save_item(NAME(m_aud_nconfig));
	save_item(NAME(m_aud_dmacntl));
	save_item(NAME(m_smrtcrd_serial_bitmask));
	save_item(NAME(m_smrtcrd_serial_rxdata));
	save_item(NAME(m_rom_cntl0));
	save_item(NAME(m_rom_cntl1));
	save_item(NAME(m_ledstate));
	save_item(NAME(dev_idcntl));
	save_item(NAME(dev_id_state));
	save_item(NAME(dev_id_bit));
	save_item(NAME(dev_id_bitidx));
}

void spot_asic_device::device_reset()
{
	dac_update_timer->adjust(attotime::from_hz(AUD_DEFAULT_CLK), 0, attotime::from_hz(AUD_DEFAULT_CLK));

	m_memcntl = 0b11;
	m_memrefcnt = 0x0400;
	m_memdata = 0x0;
	m_memtiming = 0xadbadffa;
	m_intenable = 0x0;
	m_intstat = 0x0;
	m_errenable = 0x0;
	m_chpcntl = 0x0;
	m_wdenable = 0x0;
	m_errstat = 0x0;
	m_timeout_compare = 0xffff;
	m_nvcntl = 0x0;
	m_fence1_addr = 0x0;
	m_fence1_mask = 0x0;
	m_fence2_addr = 0x0;
	m_fence2_mask = 0x0;

	m_vid_nstart = 0x0;
	m_vid_nsize = 0x0;
	m_vid_dmacntl = 0x0;
	m_vid_hstart = VID_HSTART_OFFSET + VID_DEFAULT_HSTART;
	m_vid_hsize = VID_DEFAULT_HSIZE;
	m_vid_vstart = VID_DEFAULT_VSTART;
	m_vid_vsize = VID_DEFAULT_VSIZE;
	m_vid_fcntl = 0x0;
	m_vid_blank_color = VID_DEFAULT_COLOR;
	m_vid_hintline = 0x0;
	m_vid_cstart = 0x0;
	m_vid_csize = 0x0;
	m_vid_ccnt = 0x0;
	m_vid_cline = 0x0;

	m_vid_draw_nstart = 0x0;
	m_vid_draw_hstart = VID_HSTART_OFFSET;
	m_vid_draw_hsize = m_vid_hsize;
	m_vid_draw_vstart = m_vid_vstart;
	m_vid_draw_vsize = m_vid_vsize;
	m_vid_draw_blank_color = m_vid_blank_color;

	m_aud_cstart = 0x0;
	m_aud_csize = 0x0;
	m_aud_cend = 0x0;
	m_aud_cconfig = 0x0;
	m_aud_ccnt = 0x0;
	m_aud_nstart = 0x0;
	m_aud_nsize = 0x0;
	m_aud_nconfig = 0x0;
	m_aud_dmacntl = 0x0;
	m_aud_dma_ongoing = false;

	m_rom_cntl0 = 0x0;
	m_rom_cntl1 = 0x0;

	m_ledstate = 0xFFFFFFFF;
	m_power_led = 0;
	m_connect_led = 0;
	m_message_led = 0;

	dev_idcntl = 0x00;
	dev_id_state = SSID_STATE_IDLE;
	dev_id_bit = 0x0;
	dev_id_bitidx = 0x0;

	m_smrtcrd_serial_bitmask = 0x0;
	m_smrtcrd_serial_rxdata = 0x0;

	modem_txbuff_size = 0x0;
	modem_txbuff_index = 0x0;

	spot_asic_device::validate_active_area();
	spot_asic_device::watchdog_enable(m_wdenable);
}

void spot_asic_device::device_stop()
{
}

void spot_asic_device::validate_active_area()
{
	// The active h size can't be larger than the screen width or smaller than 2 pixels.
	m_vid_draw_hsize = std::clamp(m_vid_hsize, (uint32_t)0x2, (uint32_t)m_screen->width());
	// The active v size can't be larger than the screen height or smaller than 2 pixels.
	m_vid_draw_vsize = std::clamp(m_vid_vsize, (uint32_t)0x2, (uint32_t)m_screen->height());

	// The active h start can't be smaller than 2
	m_vid_draw_hstart = std::max(m_vid_hstart - VID_HSTART_OFFSET, (uint32_t)0x2);
	// The active v start can't be smaller than 2
	m_vid_draw_vstart = std::max(m_vid_vstart, (uint32_t)0x2);

	// The active h start can't push the active area off the screen.
	if ((m_vid_draw_hstart + m_vid_draw_hsize) > m_screen->width())
		m_vid_draw_hstart = (m_screen->width() - m_vid_draw_hsize); // to screen edge
	else if (m_vid_draw_hstart < 0)
		m_vid_draw_hstart = 0;

	// The active v start can't push the active area off the screen.
	if ((m_vid_draw_vstart + m_vid_draw_vsize) > m_screen->height())
		m_vid_draw_vstart = (m_screen->height() - m_vid_draw_vsize); // to screen edge
	else if (m_vid_draw_vstart < 0)
		m_vid_draw_vstart = 0;

	spot_asic_device::pixel_buffer_index_update();
}

void spot_asic_device::pixel_buffer_index_update()
{
	uint32_t screen_lines = m_vid_draw_vsize;
	uint32_t screen_buffer_size = m_vid_nsize;

	if (m_vid_fcntl & VID_FCNTL_INTERLACE)
	{
		// Interlace mode splits the buffer into two halfs. We can capture both halfs if we double the line count.
		screen_buffer_size = (screen_buffer_size * 2);
		screen_lines = (screen_lines * 2);
	}

	m_vid_draw_nstart = m_vid_nstart;

	if (m_emu_config->read() & EMUCONFIG_PBUFF1)
	{
		m_vid_draw_nstart += screen_buffer_size;
		m_vid_draw_nstart -= (m_vid_draw_hsize * VID_BYTES_PER_PIXEL);
		m_vid_draw_vsize = screen_lines;
	}
	else
	{
		m_vid_draw_nstart += 2 * (m_vid_draw_hsize * VID_BYTES_PER_PIXEL);
		m_vid_draw_vsize = screen_lines - 3;
	}
}

void spot_asic_device::watchdog_enable(int state)
{
	m_wdenable = state;

	if (m_wdenable)
		m_watchdog->set_time(attotime::from_usec(WATCHDOG_TIMER_USEC));
	else
		m_watchdog->set_time(attotime::zero);

	m_watchdog->watchdog_enable(m_wdenable);
}

uint32_t spot_asic_device::reg_0000_r()
{
	//logerror("%s: reg_0000_r (BUS_CHIPID)\n", machine().describe_context());
	return 0x01010000;
}

uint32_t spot_asic_device::reg_0004_r()
{
	logerror("%s: reg_0004_r (BUS_CHPCNTL)\n", machine().describe_context());
	return m_chpcntl;
}

void spot_asic_device::reg_0004_w(uint32_t data)
{
	logerror("%s: reg_0004_w %08x (BUS_CHPCNTL)\n", machine().describe_context(), data);

	if ((m_chpcntl ^ data) & CHPCNTL_WDENAB_MASK)
	{
		uint32_t wd_cntl = (data & CHPCNTL_WDENAB_MASK);

		int32_t wd_diff = wd_cntl - (m_chpcntl & CHPCNTL_WDENAB_MASK);

		// Count down to disable (3, 2, 1, 0), count up to enable (0, 1, 2, 3)
		// This doesn't track the count history but gets the expected result for the ROM.
		if (
			(!m_wdenable && wd_diff > 0 && wd_cntl == CHPCNTL_WDENAB_SEQ3)
			|| (m_wdenable && wd_diff < 0 && wd_cntl == CHPCNTL_WDENAB_SEQ0)
		)
		{
			spot_asic_device::watchdog_enable(wd_cntl == CHPCNTL_WDENAB_SEQ3);
		}
	}

	if (!(m_sys_config->read() & SYSCONFIG_AUDDACMODE) && (m_chpcntl ^ data) & CHPCNTL_AUDCLKDIV_MASK)
	{
		uint32_t audclk_cntl = (data & CHPCNTL_AUDCLKDIV_MASK);

		uint32_t sys_clk = spot_asic_device::clock();
		uint32_t aud_clk = AUD_DEFAULT_CLK;

		switch(audclk_cntl)
		{
			case CHPCNTL_AUDCLKDIV_EXTC:
			default:
				aud_clk = AUD_DEFAULT_CLK;
				break;

			case CHPCNTL_AUDCLKDIV_DIV1:
				aud_clk = sys_clk / (1 * 0x100);
				break;

			case CHPCNTL_AUDCLKDIV_DIV2:
				aud_clk = sys_clk / (2 * 0x100);
				break;

			case CHPCNTL_AUDCLKDIV_DIV3:
				aud_clk = sys_clk / (3 * 0x100);
				break;

			case CHPCNTL_AUDCLKDIV_DIV4:
				aud_clk = sys_clk / (4 * 0x100);
				break;

			case CHPCNTL_AUDCLKDIV_DIV5:
				aud_clk = sys_clk / (5 * 0x100);
				break;

			case CHPCNTL_AUDCLKDIV_DIV6:
				aud_clk = sys_clk / (6 * 0x100);
				break;

		}

		dac_update_timer->adjust(attotime::from_hz(aud_clk), 0, attotime::from_hz(aud_clk));
	}

	m_chpcntl = data;
}

uint32_t spot_asic_device::reg_0008_r()
{
	logerror("%s: reg_0008_r (BUS_INTSTAT)\n", machine().describe_context());
	if (m_intstat == 0x0)
		return BUS_INT_VIDINT;
	else
		return m_intstat;
}

void spot_asic_device::reg_0108_w(uint32_t data)
{
	logerror("%s: reg_0108_w %08x (BUS_INTSTAT clear)\n", machine().describe_context(), data);

	spot_asic_device::set_bus_irq(data, 0);
}

uint32_t spot_asic_device::reg_000c_r()
{
	logerror("%s: reg_000c_r (BUS_INTEN)\n", machine().describe_context());
	return m_intenable;
}

void spot_asic_device::reg_000c_w(uint32_t data)
{
	logerror("%s: reg_000c_w %08x (BUS_INTEN)\n", machine().describe_context(), data);
	if (data&BUS_INT_AUDDMA)
		logerror("%s: AUDDMA bus interrupt set\n", machine().describe_context());
	if (data&BUS_INT_DEVSMC)
		logerror("%s: DEVSMC bus interrupt set\n", machine().describe_context());
	if (data&BUS_INT_DEVIR)
		logerror("%s: DEVIR bus interrupt set\n", machine().describe_context());
	if (data&BUS_INT_DEVMOD)
		logerror("%s: DEVMOD bus interrupt set\n", machine().describe_context());
	if (data&BUS_INT_DEVKBD)
		logerror("%s: DEVKBD bus interrupt set\n", machine().describe_context());
	if (data&BUS_INT_VIDINT)
		logerror("%s: VIDINT bus interrupt set\n", machine().describe_context());

	m_intenable |= data & 0xFF;
}

void spot_asic_device::reg_010c_w(uint32_t data)
{
	logerror("%s: reg_010c_w %08x (BUS_INTEN clear)\n", machine().describe_context(), data);
	if (data&BUS_INT_AUDDMA)
		logerror("%s: AUDDMA bus interrupt cleared\n", machine().describe_context());
	if (data&BUS_INT_DEVSMC)
		logerror("%s: DEVSMC bus interrupt cleared\n", machine().describe_context());
	if (data&BUS_INT_DEVIR)
		logerror("%s: DEVIR bus interrupt cleared\n", machine().describe_context());
	if (data&BUS_INT_DEVMOD)
		logerror("%s: DEVMOD bus interrupt cleared\n", machine().describe_context());
	if (data&BUS_INT_DEVKBD)
		logerror("%s: DEVKBD bus interrupt cleared\n", machine().describe_context());
	if (data&BUS_INT_VIDINT)
		logerror("%s: VIDINT bus interrupt cleared\n", machine().describe_context());

	if (data != BUS_INT_DEVMOD) // The modem timinng is incorrect, so ignore the ROM trying to disable the modem interrupt.
		m_intenable &= ~(data & 0xFF);
}

uint32_t spot_asic_device::reg_0010_r()
{
	logerror("%s: reg_0010_r (BUS_ERRSTAT)\n", machine().describe_context());
	return m_errstat;
}

void spot_asic_device::reg_0110_w(uint32_t data)
{
	logerror("%s: reg_0110_w %08x (BUS_ERRSTAT clear)\n", machine().describe_context(), data);
	m_errstat &= (~data) & 0xFF;
}

uint32_t spot_asic_device::reg_0014_r()
{
	logerror("%s: reg_0014_r (BUS_ERREN)\n", machine().describe_context());
	return m_errenable;
}

void spot_asic_device::reg_0014_w(uint32_t data)
{
	logerror("%s: reg_0014_w %08x (BUS_ERREN set)\n", machine().describe_context(), data);
	m_errenable |= data & 0xFF;
}

void spot_asic_device::reg_0114_w(uint32_t data)
{
	logerror("%s: reg_0114_w %08x (BUS_ERREN clear)\n", machine().describe_context(), data);
	m_errenable &= (~data) & 0xFF;
}

uint32_t spot_asic_device::reg_0018_r()
{
	logerror("%s: reg_0018_r (BUS_ERRADDR)\n", machine().describe_context());
	return 0x00000000;
}

void spot_asic_device::reg_0118_w(uint32_t data)
{
	logerror("%s: reg_0118_w %08x (BUS_WDREG clear)\n", machine().describe_context(), data);
	if (m_wdenable)
		m_watchdog->reset_w(data);
}

uint32_t spot_asic_device::reg_001c_r()
{
	logerror("%s: reg_001c_r (BUS_FENADDR1)\n", machine().describe_context());
	return m_fence1_addr;
}

void spot_asic_device::reg_001c_w(uint32_t data)
{
	logerror("%s: reg_001c_w %08x (BUS_FENADDR1)\n", machine().describe_context(), data);
	m_fence1_addr = data;
}

uint32_t spot_asic_device::reg_0020_r()
{
	logerror("%s: reg_0020_r (BUS_FENMASK1)\n", machine().describe_context());
	return m_fence1_mask;
}

void spot_asic_device::reg_0020_w(uint32_t data)
{
	logerror("%s: reg_0020_w %08x (BUS_FENMASK1)\n", machine().describe_context(), data);
	m_fence1_mask = data;
}

uint32_t spot_asic_device::reg_0024_r()
{
	logerror("%s: reg_0024_r (BUS_FENADDR2)\n", machine().describe_context());
	return m_fence2_addr;
}

void spot_asic_device::reg_0024_w(uint32_t data)
{
	logerror("%s: reg_0024_w %08x (BUS_FENADDR2)\n", machine().describe_context(), data);
	m_fence2_addr = data;
}

uint32_t spot_asic_device::reg_0028_r()
{
	logerror("%s: reg_0028_r (BUS_FENMASK2)\n", machine().describe_context());
	return m_fence2_mask;
}

void spot_asic_device::reg_0028_w(uint32_t data)
{
	logerror("%s: reg_0028_w %08x (BUS_FENMASK2)\n", machine().describe_context(), data);
	m_fence2_mask = data;
}

uint32_t spot_asic_device::reg_1000_r()
{
	logerror("%s: reg_1000_r (ROM_SYSCONF)\n", machine().describe_context());
	return m_sys_config->read();
}

uint32_t spot_asic_device::reg_1004_r()
{
	logerror("%s: reg_1004_r (ROM_CNTL0)\n", machine().describe_context());
	return m_rom_cntl0;
}

void spot_asic_device::reg_1004_w(uint32_t data)
{
	logerror("%s: reg_1004_w %08x (ROM_CNTL0)\n", machine().describe_context(), data);
	m_rom_cntl0 = data;
}

uint32_t spot_asic_device::reg_1008_r()
{
	logerror("%s: reg_1008_r (ROM_CNTL1)\n", machine().describe_context());
	return m_rom_cntl1;
}

void spot_asic_device::reg_1008_w(uint32_t data)
{
	logerror("%s: reg_1008_w %08x (ROM_CNTL1)\n", machine().describe_context(), data);
	m_rom_cntl1 = data;
}

uint32_t spot_asic_device::reg_2000_r()
{
	logerror("%s: reg_2000_r (AUD_CSTART)\n", machine().describe_context());
	return m_aud_cstart;
}

uint32_t spot_asic_device::reg_2004_r()
{
	logerror("%s: reg_2004_r (AUD_CSIZE)\n", machine().describe_context());
	return m_aud_csize;
}

uint32_t spot_asic_device::reg_2008_r()
{
	logerror("%s: reg_2008_r (AUD_CCONFIG)\n", machine().describe_context());
	return m_aud_cconfig;
}

void spot_asic_device::reg_2008_w(uint32_t data)
{
	logerror("%s: reg_2008_w %08x (AUD_CCONFIG)\n", machine().describe_context(), data);
	m_aud_cconfig = data;
}

uint32_t spot_asic_device::reg_200c_r()
{
	logerror("%s: reg_200c_r (AUD_CCNT)\n", machine().describe_context());
	return m_aud_ccnt;
}

uint32_t spot_asic_device::reg_2010_r()
{
	//logerror("%s: reg_2010_r (AUD_NSTART)\n", machine().describe_context());
	return m_aud_nstart;
}

void spot_asic_device::reg_2010_w(uint32_t data)
{
	//logerror("%s: reg_2010_w %08x (AUD_NSTART)\n", machine().describe_context(), data);
	m_aud_nstart = data;
}

uint32_t spot_asic_device::reg_2014_r()
{
	//logerror("%s: reg_2014_r (AUD_NSIZE)\n", machine().describe_context());
	return m_aud_nsize;
}

void spot_asic_device::reg_2014_w(uint32_t data)
{
	//logerror("%s: reg_2014_w %08x (AUD_NSIZE)\n", machine().describe_context(), data);

	m_aud_nsize = data;
}

uint32_t spot_asic_device::reg_2018_r()
{
	//logerror("%s: reg_2018_r (AUD_NCONFIG)\n", machine().describe_context());
	return m_aud_nconfig;
}

void spot_asic_device::reg_2018_w(uint32_t data)
{
	//logerror("%s: reg_2018_w %08x (AUD_NCONFIG)\n", machine().describe_context(), data);
	m_aud_nconfig = data;
}

uint32_t spot_asic_device::reg_201c_r()
{
	//logerror("%s: reg_201c_r (AUD_DMACNTL)\n", machine().describe_context());

	spot_asic_device::irq_audio_w(0);

	return m_aud_dmacntl;
}

void spot_asic_device::reg_201c_w(uint32_t data)
{
    //logerror("%s: reg_201c_w %08x (AUD_DMACNTL)\n", machine().describe_context(), data);
	if ((m_aud_dmacntl ^ data) & AUD_DMACNTL_DMAEN)
	{
		if (data & AUD_DMACNTL_DMAEN)
		{
			m_lspeaker->set_input_gain(0, AUD_OUTPUT_GAIN);
			m_rspeaker->set_input_gain(0, AUD_OUTPUT_GAIN);
		}
		else
		{
			m_lspeaker->set_input_gain(0, 0.0);
			m_rspeaker->set_input_gain(0, 0.0);
		}
	}

	m_aud_dmacntl = data;
}

uint32_t spot_asic_device::reg_3000_r()
{
	//logerror("%s: reg_3000_r (VID_CSTART)\n", machine().describe_context());
	return m_vid_cstart;
}

uint32_t spot_asic_device::reg_3004_r()
{
	//logerror("%s: reg_3004_r (VID_CSIZE)\n", machine().describe_context());
	return m_vid_csize;
}

uint32_t spot_asic_device::reg_3008_r()
{
	//logerror("%s: reg_3008_r (VID_CCNT)\n", machine().describe_context());
	return m_vid_ccnt;
}

uint32_t spot_asic_device::reg_300c_r()
{
	//logerror("%s: reg_300c_r (VID_NSTART)\n", machine().describe_context());
	return m_vid_nstart;
}

void spot_asic_device::reg_300c_w(uint32_t data)
{
	bool has_changed = (m_vid_nstart != data);

	m_vid_nstart = data;

	if (has_changed)
		spot_asic_device::validate_active_area();

	//logerror("%s: reg_300c_w %08x (VID_NSTART)\n", machine().describe_context(), data);
}

uint32_t spot_asic_device::reg_3010_r()
{
	//logerror("%s: reg_3010_r (VID_NSIZE)\n", machine().describe_context());
	return m_vid_nsize;
}

void spot_asic_device::reg_3010_w(uint32_t data)
{
	bool has_changed = (m_vid_nsize != data);

	m_vid_nsize = data;

	if (has_changed)
		spot_asic_device::validate_active_area();

	//logerror("%s: reg_3010_w %08x (VID_NSIZE)\n", machine().describe_context(), data);
}

uint32_t spot_asic_device::reg_3014_r()
{
	//logerror("%s: reg_3014_r (VID_DMACNTL)\n", machine().describe_context());
	return m_vid_dmacntl;
}

void spot_asic_device::reg_3014_w(uint32_t data)
{
	if ((m_vid_dmacntl ^ data) & VID_DMACNTL_NV && data & VID_DMACNTL_NV)
		spot_asic_device::pixel_buffer_index_update();

	m_vid_dmacntl = data;

	//logerror("%s: reg_3014_w %08x (VID_DMACNTL)\n", machine().describe_context(), data);
}

uint32_t spot_asic_device::reg_3018_r()
{
	//logerror("%s: reg_3018_r (VID_FCNTL)\n", machine().describe_context());
	return m_vid_fcntl;
}

void spot_asic_device::reg_3018_w(uint32_t data)
{
	m_vid_fcntl = data;

	//logerror("%s: reg_3018_w %08x (VID_FCNTL)\n", machine().describe_context(), data);
}

uint32_t spot_asic_device::reg_301c_r()
{
	logerror("%s: reg_301c_r (VID_BLNKCOL)\n", machine().describe_context());
	return m_vid_blank_color;
}

void spot_asic_device::reg_301c_w(uint32_t data)
{
	logerror("%s: reg_301c_r %08x (VID_BLNKCOL)\n", machine().describe_context(), data);
	m_vid_blank_color = data;

	m_vid_draw_blank_color = (((data >> 0x10) & 0xff) << 0x18) | (((data >> 0x08) & 0xff) << 0x10) | (((data >> 0x10) & 0xff) << 0x08) | (data & 0xff);	
}

uint32_t spot_asic_device::reg_3020_r()
{
	//logerror("%s: reg_3020_r (VID_HSTART)\n", machine().describe_context());
	return m_vid_hstart;
}

void spot_asic_device::reg_3020_w(uint32_t data)
{
	bool has_changed = (m_vid_hstart != data);

	m_vid_hstart = data;

	if (has_changed)
		spot_asic_device::validate_active_area();

	//logerror("%s: reg_3020_w %08x (VID_HSTART)\n", machine().describe_context(), data);
}

uint32_t spot_asic_device::reg_3024_r()
{
	//logerror("%s: reg_3024_r (VID_HSIZE)\n", machine().describe_context());
	return m_vid_hsize;
}

void spot_asic_device::reg_3024_w(uint32_t data)
{
	bool has_changed = (m_vid_hsize != data);

	m_vid_hsize = data;

	if (has_changed)
		spot_asic_device::validate_active_area();

	//logerror("%s: reg_3024_w %08x (VID_HSIZE)\n", machine().describe_context(), data);
}

uint32_t spot_asic_device::reg_3028_r()
{
	//logerror("%s: reg_3028_r (VID_VSTART)\n", machine().describe_context());
	return m_vid_vstart;
}

void spot_asic_device::reg_3028_w(uint32_t data)
{
	bool has_changed = (m_vid_vstart != data);

	m_vid_vstart = data;

	if (has_changed)
		spot_asic_device::validate_active_area();

	//logerror("%s: reg_3028_w %08x (VID_VSTART)\n", machine().describe_context(), data);
}

uint32_t spot_asic_device::reg_302c_r()
{
	//logerror("%s: reg_302c_r (VID_VSIZE)\n", machine().describe_context());
	return m_vid_vsize;
}

void spot_asic_device::reg_302c_w(uint32_t data)
{
	bool has_changed = (m_vid_vstart != data);

	m_vid_vsize = data;

	if (has_changed)
		spot_asic_device::validate_active_area();

	//logerror("%s: reg_302c_w %08x (VID_VSIZE)\n", machine().describe_context(), data);
}

uint32_t spot_asic_device::reg_3030_r()
{
	//logerror("%s: reg_3030_r (VID_HINTLINE)\n", machine().describe_context());
	return m_vid_hintline;
}

void spot_asic_device::reg_3030_w(uint32_t data)
{
	m_vid_hintline = data;
	//logerror("%s: reg_3030_w %08x (VID_HINTLINE)\n", machine().describe_context(), data);
}

uint32_t spot_asic_device::reg_3034_r()
{
	//logerror("%s: reg_3034_r (VID_CLINE)\n", machine().describe_context());

	return m_screen->vpos();
	//return (m_vid_cline++) & 0x1ffff;
}

uint32_t spot_asic_device::reg_3038_r()
{
	//logerror("%s: reg_3038_r (VID_INTSTAT read)\n", machine().describe_context());
	return m_vid_intstat;
}

void spot_asic_device::reg_3138_w(uint32_t data)
{
	//logerror("%s: reg_3138_w %08x (VID_INTSTAT clear)\n", machine().describe_context(), data);
	m_vid_intstat &= (~data) & 0xff;
}

uint32_t spot_asic_device::reg_303c_r()
{
	logerror("%s: reg_303c_r (VID_INTEN_S)\n", machine().describe_context());
	return m_vid_intenable;
}

void spot_asic_device::reg_303c_w(uint32_t data)
{
	logerror("%s: reg_303c_w %08x (VID_INTEN_S)\n", machine().describe_context(), data);
	m_vid_intenable |= (data & 0xff);
}

void spot_asic_device::reg_313c_w(uint32_t data)
{
	logerror("%s: reg_313c_w %08x (VID_INTEN_C clear)\n", machine().describe_context(), data);
	 m_vid_intenable &= (~data) & 0xff;
}

// Read IR receiver chip
uint32_t spot_asic_device::reg_4000_r()
{
	logerror("%s: reg_4000_r (DEV_IRDATA)\n", machine().describe_context());
	// TODO: This seems to have been handled by a PIC16CR54AT. We do not have the ROM for this chip, so its behavior will need to be emulated at a high level.
	return 0;
}

// Read LED states
uint32_t spot_asic_device::reg_4004_r()
{
    logerror("%s: reg_4004_r (DEV_LED)\n", machine().describe_context());
    m_power_led = !BIT(m_ledstate, 2);
    m_connect_led = !BIT(m_ledstate, 1);
    m_message_led = !BIT(m_ledstate, 0);
    return m_ledstate;
}

// Update LED states
void spot_asic_device::reg_4004_w(uint32_t data)
{
	logerror("%s: reg_4004_w %08x (DEV_LED)\n", machine().describe_context(), data);
	m_ledstate = data;
	m_power_led = !BIT(m_ledstate, 2);
	m_connect_led = !BIT(m_ledstate, 1);
	m_message_led = !BIT(m_ledstate, 0);
}

// Not using logic inside DS2401.cpp because the delay logic in the ROM doesn't work properly.

uint32_t spot_asic_device::reg_4008_r()
{
	dev_id_bit = 0x0;

	if (dev_id_state == SSID_STATE_PRESENCE)
	{
		dev_id_bit = 0x0; // We're present.
		dev_id_state = SSID_STATE_COMMAND; // This normally would stay in presence mode for 480us then command, but we immediatly go into command mode.
		dev_id_bitidx = 0x0;
	}
	else if (dev_id_state == SSID_STATE_READROM_PULSEEND)
	{
		dev_id_state = SSID_STATE_READROM_BIT;
	}
	else if (dev_id_state == SSID_STATE_READROM_BIT)
	{
		dev_id_state = SSID_STATE_READROM; // Go back into the read ROM pulse state

		dev_id_bit = m_serial_id->direct_read(dev_id_bitidx / 8) >> (dev_id_bitidx & 0x7);

		dev_id_bitidx++;
		if (dev_id_bitidx == 64)
		{
			// We've read the entire SSID. Go back into idle.
			dev_id_state = SSID_STATE_IDLE;
			dev_id_bitidx = 0x0;
		}
	}

	return dev_idcntl | (dev_id_bit & 1);
}

void spot_asic_device::reg_4008_w(uint32_t data)
{
	dev_idcntl = (data & 0x2);

	if (dev_idcntl & 0x2)
	{
		switch(dev_id_state) // States for high
		{
			case SSID_STATE_RESET: // End reset low pulse to go into prescense mode. Chip should read low to indicate presence.
				dev_id_state = SSID_STATE_PRESENCE; // This pulse normally lasts 480us before going into command mode.
				break;

			case SSID_STATE_COMMAND: // Ended a command bit pulse. Increment bit index. We always assume a read from ROM command after we get 8 bits.
				dev_id_bitidx++;

				if (dev_id_bitidx == 8)
				{
					dev_id_state = SSID_STATE_READROM; // Now we can read back the SSID. ROM reads it as two 32-bit integers.
					dev_id_bitidx = 0;
				}
				break;

			case SSID_STATE_READROM_PULSESTART:
				dev_id_state = SSID_STATE_READROM_PULSEEND;
		}
	}
	else
	{
		switch(dev_id_state) // States for low
		{
			case SSID_STATE_IDLE: // When idle, we can drive the chip low for reset
				dev_id_state = SSID_STATE_RESET; // We'd normally leave this for 480us to go into presence mode.
				break;

			case SSID_STATE_READROM:
				dev_id_state = SSID_STATE_READROM_PULSESTART;
				break;
		}
	}
}

// 400c commands the I2C bus (referenced as the IIC bus in WebTV's code)
//
// The SPOT programming doc calls this as an NVCNTL register but this us used as an I2C register.
//
// There's two known devices that sit on this bus:
//
//	Address		Device
//	0x8C		Philips SAA7187 encoder
//				Used for the S-Video and composite out
//	0xa0		Atmel AT24C01A EEPROM NVRAM
//				Used for the encryption shared secret (0x14) and crash log counter (0x23)
//
// We emulate the AT24C01A here.
//
uint32_t spot_asic_device::reg_400c_r()
{
	int sda_bit = (m_nvram->read_sda()) & 0x1;

	return (m_nvcntl & 0xE) | sda_bit;
}

void spot_asic_device::reg_400c_w(uint32_t data)
{
	if (data & NVCNTL_WRITE_EN) {
		m_nvram->write_sda(((data & NVCNTL_SDA_W) == NVCNTL_SDA_W) & 0x1);
	} else {
		m_nvram->write_sda(0x1);
	}

	m_nvram->write_scl(((data & NVCNTL_SCL) == NVCNTL_SCL) & 1);

	m_nvcntl = data & 0xE;
}

uint32_t spot_asic_device::reg_4010_r()
{
	logerror("%s: reg_4010_r (DEV_SCCNTL)\n", machine().describe_context());
	if (m_emu_config->read() & EMUCONFIG_BANGSERIAL)
	{
		// bitbang functionality does not accept smartcard input
		return 0;
	}
	else
	{
		// TODO: get data!
		return 0;
	}
}

void spot_asic_device::reg_4010_w(uint32_t data)
{
	// TODO: This should be refactored into its own serial device that plugs into the SmartCard slot.
	if (m_emu_config->read() & EMUCONFIG_BANGSERIAL)
	{
		m_smrtcrd_serial_bitmask = (m_smrtcrd_serial_bitmask << 1) | 1;
		m_smrtcrd_serial_rxdata = (m_smrtcrd_serial_rxdata << 1) | (data == 0);

		// Just checking if the all bits are present. Not checking if they're valid.
		if ((m_smrtcrd_serial_bitmask & 0x7ff) == 0x7ff)
		{
			uint8_t bangserial_config = (m_emu_config->read() & EMUCONFIG_BANGSERIAL);
			uint8_t rxbyte = 0x00;

			if ((bangserial_config == EMUCONFIG_BANGSERIAL_AUTO && ((m_smrtcrd_serial_rxdata & 0x700) != 0x600)) || (bangserial_config == EMUCONFIG_BANGSERIAL_V1))
				// V1: there's 2 bits at the start (1 high and 1 low), 8 data bits and 1 bit at the end.
				rxbyte = (m_smrtcrd_serial_rxdata >> 1);
			else
				// V2: there's 3 bits at the start (all high), 8 data bits and no bits at the end.
				rxbyte = m_smrtcrd_serial_rxdata;

			// This reverses the bit order
			rxbyte = (rxbyte & 0xf0) >> 4 | (rxbyte & 0x0f) << 4; // Divide byte into 2 nibbles and swap them
			rxbyte = (rxbyte & 0xcc) >> 2 | (rxbyte & 0x33) << 2; // Divide nibble into 2 bits and swap them
			rxbyte = (rxbyte & 0xaa) >> 1 | (rxbyte & 0x55) << 1; // Divide again and swap the remaining bits

			osd_printf_verbose("%c", rxbyte);

			m_smrtcrd_serial_bitmask = 0x0;
			m_smrtcrd_serial_rxdata = 0x0;
		}
	}
	else
	{
		// TODO: reimplement smartcard slot
	}

	logerror("%s: reg_4010_w %08x (DEV_SCCNTL)\n", machine().describe_context(), data);
}

uint32_t spot_asic_device::reg_4014_r()
{
	//logerror("%s: reg_4014_r (DEV_EXTTIME)\n", machine().describe_context());
	return 0;
}

void spot_asic_device::reg_4014_w(uint32_t data)
{
	//logerror("%s: reg_4014_w %08x (DEV_EXTTIME)\n", machine().describe_context(), data);
}

uint32_t spot_asic_device::reg_4020_r()
{
	logerror("%s: reg_4020_r (DEV_KBD0)\n", machine().describe_context());
	return m_kbdc->data_r(0x0);
}

void spot_asic_device::reg_4020_w(uint32_t data)
{
	logerror("%s: reg_4020_w %08x (DEV_KBD0)\n", machine().describe_context(), data);
	m_kbdc->data_w(0x0, data & 0xFF);
}

uint32_t spot_asic_device::reg_4024_r()
{
	logerror("%s: reg_4024_r (DEV_KBD1)\n", machine().describe_context());
	return m_kbdc->data_r(0x4);
}

void spot_asic_device::reg_4024_w(uint32_t data)
{
	logerror("%s: reg_4024_w %08x (DEV_KBD1)\n", machine().describe_context(), data);
	m_kbdc->data_w(0x4, data & 0xFF);
}

uint32_t spot_asic_device::reg_4028_r()
{
	logerror("%s: reg_4028_r (DEV_KBD2)\n", machine().describe_context());
	return m_kbdc->data_r(0x2);
}

void spot_asic_device::reg_4028_w(uint32_t data)
{
	logerror("%s: reg_4028_w %08x (DEV_KBD2)\n", machine().describe_context(), data);
	m_kbdc->data_w(0x2, data & 0xFF);
}

uint32_t spot_asic_device::reg_402c_r()
{
	logerror("%s: reg_402c_r (DEV_KBD3)\n", machine().describe_context());
	return m_kbdc->data_r(0x3);
}

void spot_asic_device::reg_402c_w(uint32_t data)
{
	logerror("%s: reg_402c_w %08x (DEV_KBD3)\n", machine().describe_context(), data);
	m_kbdc->data_w(0x3, data & 0xFF);
}

uint32_t spot_asic_device::reg_4030_r()
{
	logerror("%s: reg_4030_r (DEV_KBD4)\n", machine().describe_context());
	return m_kbdc->data_r(0x1);
}

void spot_asic_device::reg_4030_w(uint32_t data)
{
	logerror("%s: reg_4030_w %08x (DEV_KBD4)\n", machine().describe_context(), data);
	m_kbdc->data_w(0x1, data & 0xFF);
}

uint32_t spot_asic_device::reg_4034_r()
{
	logerror("%s: reg_4034_r (DEV_KBD5)\n", machine().describe_context());
	return m_kbdc->data_r(0x5);
}

void spot_asic_device::reg_4034_w(uint32_t data)
{
	logerror("%s: reg_4034_w %08x (DEV_KBD5)\n", machine().describe_context(), data);
	m_kbdc->data_w(0x5, data & 0xFF);
}

uint32_t spot_asic_device::reg_4038_r()
{
	logerror("%s: reg_4038_r (DEV_KBD6)\n", machine().describe_context());
	return m_kbdc->data_r(0x6);
}

void spot_asic_device::reg_4038_w(uint32_t data)
{
	logerror("%s: reg_4038_w %08x (DEV_KBD6)\n", machine().describe_context(), data);
	m_kbdc->data_w(0x6, data & 0xFF);
}

uint32_t spot_asic_device::reg_403c_r()
{
	logerror("%s: reg_403c_r (DEV_KBD7)\n", machine().describe_context());
	return m_kbdc->data_r(0x7);
}

void spot_asic_device::reg_403c_w(uint32_t data)
{
	logerror("%s: reg_403c_w %08x (DEV_KBD7)\n", machine().describe_context(), data);
	m_kbdc->data_w(0x7, data & 0xFF);
}

uint32_t spot_asic_device::reg_4040_r()
{
	logerror("%s: reg_4040_r (DEV_MOD0)\n", machine().describe_context());
	return m_modem_uart->ins8250_r(0x0);
}

void spot_asic_device::reg_4040_w(uint32_t data)
{
	logerror("%s: reg_4040_w %08x (DEV_MOD0)\n", machine().describe_context(), data);

	if (modem_txbuff_size == 0 && (m_modem_uart->ins8250_r(0x5) & INS8250_LSR_TSRE))
	{
		m_modem_uart->ins8250_w(0x0, data & 0xFF);
	}
	else
	{
		modem_txbuff[modem_txbuff_size++ & (MBUFF_MAX_SIZE - 1)] = data & 0xFF;

		modem_buffer_timer->adjust(attotime::from_usec(MBUFF_FLUSH_TIME));
	}
}

uint32_t spot_asic_device::reg_4044_r()
{
	logerror("%s: reg_4044_r (DEV_MOD1)\n", machine().describe_context());
	return m_modem_uart->ins8250_r(0x1);
}

void spot_asic_device::reg_4044_w(uint32_t data)
{
	logerror("%s: reg_4044_w %08x (DEV_MOD1)\n", machine().describe_context(), data);
	m_modem_uart->ins8250_w(0x1, data & 0xFF);
}

uint32_t spot_asic_device::reg_4048_r()
{
	logerror("%s: reg_4048_r (DEV_MOD2)\n", machine().describe_context());
	return m_modem_uart->ins8250_r(0x2);
}

void spot_asic_device::reg_4048_w(uint32_t data)
{
	logerror("%s: reg_4048_w %08x (DEV_MOD2)\n", machine().describe_context(), data);
	m_modem_uart->ins8250_w(0x2, data & 0xFF);
}

uint32_t spot_asic_device::reg_404c_r()
{
	logerror("%s: reg_404c_r (DEV_MOD3)\n", machine().describe_context());
	return m_modem_uart->ins8250_r(0x3);
}

void spot_asic_device::reg_404c_w(uint32_t data)
{
	logerror("%s: reg_404c_w %08x (DEV_MOD3)\n", machine().describe_context(), data);
	m_modem_uart->ins8250_w(0x3, data & 0xFF);
}

uint32_t spot_asic_device::reg_4050_r()
{
	logerror("%s: reg_4050_r (DEV_MOD4)\n", machine().describe_context());
	return m_modem_uart->ins8250_r(0x4);
}

void spot_asic_device::reg_4050_w(uint32_t data)
{
	logerror("%s: reg_4050_w %08x (DEV_MOD4)\n", machine().describe_context(), data);
	m_modem_uart->ins8250_w(0x4, data & 0xFF);
}

uint32_t spot_asic_device::reg_4054_r()
{
	logerror("%s: reg_4054_r (DEV_MOD5)\n", machine().describe_context());
	return m_modem_uart->ins8250_r(0x5);
}

void spot_asic_device::reg_4054_w(uint32_t data)
{
	logerror("%s: reg_4054_w %08x (DEV_MOD5)\n", machine().describe_context(), data);
	m_modem_uart->ins8250_w(0x5, data & 0xFF);
}

uint32_t spot_asic_device::reg_4058_r()
{
	logerror("%s: reg_4058_r (DEV_MOD6)\n", machine().describe_context());
	return m_modem_uart->ins8250_r(0x6);
}

void spot_asic_device::reg_4058_w(uint32_t data)
{
	logerror("%s: reg_4058_w %08x (DEV_MOD6)\n", machine().describe_context(), data);
	m_modem_uart->ins8250_w(0x6, data & 0xFF);
}

uint32_t spot_asic_device::reg_405c_r()
{
	logerror("%s: reg_405c_r (DEV_MOD7)\n", machine().describe_context());
	return m_modem_uart->ins8250_r(0x7);
}

void spot_asic_device::reg_405c_w(uint32_t data)
{
	logerror("%s: reg_405c_w %08x (DEV_MOD7)\n", machine().describe_context(), data);
	m_modem_uart->ins8250_w(0x7, data & 0xFF);
}

// memUnit registers

uint32_t spot_asic_device::reg_5000_r()
{
	logerror("%s: reg_5000_r (MEM_CNTL)\n", machine().describe_context());
	return m_memcntl;
}

void spot_asic_device::reg_5000_w(uint32_t data)
{
	logerror("%s: reg_5000_w %08x (MEM_CNTL)\n", machine().describe_context(), data);
	m_memcntl = data;
}

uint32_t spot_asic_device::reg_5004_r()
{
	logerror("%s: reg_5004_r (MEM_REFCNT)\n", machine().describe_context());
	return m_memrefcnt;
}

void spot_asic_device::reg_5004_w(uint32_t data)
{
	logerror("%s: reg_5004_w %08x (MEM_REFCNT)\n", machine().describe_context(), data);
	m_memrefcnt = data;
}

uint32_t spot_asic_device::reg_5008_r()
{
	logerror("%s: reg_5008_r (MEM_DATA)\n", machine().describe_context());
	return m_memdata;
}

void spot_asic_device::reg_5008_w(uint32_t data)
{
	logerror("%s: reg_5008_w %08x (MEM_DATA)\n", machine().describe_context(), data);
	m_memdata = data;
}

/*uint32_t spot_asic_device::reg_500c_r()
{
	logerror("%s: reg_500c_r (MEM_CMD - not a readable register!)\n", machine().describe_context());
	// FIXME: This is defined as a write-only register, yet the WebTV software reads from it? Still need to see what the software expects from this.
	return 0;
}*/

void spot_asic_device::reg_500c_w(uint32_t data)
{
	//logerror("%s: reg_500c_w %08x (MEM_CMD)\n", machine().describe_context(), data);
}

uint32_t spot_asic_device::reg_5010_r()
{
	//logerror("%s: reg_5010_r (MEM_TIMING)\n", machine().describe_context());
	return m_memtiming;
}

void spot_asic_device::reg_5010_w(uint32_t data)
{
	logerror("%s: reg_500c_w %08x (MEM_TIMING)\n", machine().describe_context(), data);
	m_memtiming = data;
}

TIMER_CALLBACK_MEMBER(spot_asic_device::dac_update)
{
	if (m_aud_dmacntl & AUD_DMACNTL_DMAEN)
	{
		if (m_aud_dma_ongoing)
		{
			address_space &space = m_hostcpu->space(AS_PROGRAM);

			int16_t samplel = space.read_word(m_aud_ccnt);
			m_aud_ccnt += 2;
			int16_t sampler = space.read_word(m_aud_ccnt);
			m_aud_ccnt += 2;
			// For 8-bit we're assuming left-aligned samples
			switch(m_aud_cconfig)
			{
				case AUD_CONFIG_16BIT_STEREO:
				default:
					m_dac[0]->write(samplel);
					m_dac[1]->write(sampler);
					break;

				case AUD_CONFIG_16BIT_MONO:
					m_dac[0]->write(samplel);
					m_dac[1]->write(samplel);
					break;

				case AUD_CONFIG_8BIT_STEREO:
					m_dac[0]->write((samplel >> 0x8) & 0xFF);
					m_dac[1]->write((sampler >> 0x8) & 0xFF);
					break;

				case AUD_CONFIG_8BIT_MONO:
					m_dac[0]->write((samplel >> 0x8) & 0xFF);
					m_dac[1]->write((samplel >> 0x8) & 0xFF);
					break;
			}
			if (m_aud_ccnt >= m_aud_cend)
			{
				spot_asic_device::irq_audio_w(1);
				m_aud_dma_ongoing = false; // nothing more to DMA
			}
		}
		if (!m_aud_dma_ongoing)
		{
			// wait for next DMA values to be marked as valid
			m_aud_dma_ongoing = m_aud_dmacntl & (AUD_DMACNTL_NV | AUD_DMACNTL_NVF);
			if (!m_aud_dma_ongoing) return; // values aren't marked as valid; don't prepare for next DMA
			m_aud_cstart = m_aud_nstart;
			m_aud_csize = m_aud_nsize;
			m_aud_cend = (m_aud_cstart + m_aud_csize);
			m_aud_cconfig = m_aud_nconfig;
			m_aud_ccnt = m_aud_cstart;
		}
	}
}

TIMER_CALLBACK_MEMBER(spot_asic_device::flush_modem_buffer)
{
	if (modem_txbuff_size > 0 && (m_modem_uart->ins8250_r(0x5) & INS8250_LSR_TSRE))
	{
		m_modem_uart->ins8250_w(0x0, modem_txbuff[modem_txbuff_index++ & (MBUFF_MAX_SIZE - 1)]);

		if (modem_txbuff_index == modem_txbuff_size)
		{
			modem_txbuff_index = 0x0;
			modem_txbuff_size = 0x0;
		}
	}

	if (modem_txbuff_size > 0)
		modem_buffer_timer->adjust(attotime::from_usec(MBUFF_FLUSH_TIME));
}

// The interrupt handler gets copied into memory @ 0x80000200 to match up with the MIPS3 interrupt vector

void spot_asic_device::vblank_irq(int state) 
{
	// Not to spec but does get the intended result.
	// All video interrupts are classed the same in the ROM.
	spot_asic_device::set_vid_irq(VID_INT_VSYNCO, 1);
}

void spot_asic_device::irq_keyboard_w(int state)
{
	spot_asic_device::set_bus_irq(BUS_INT_DEVKBD, state);
}

void spot_asic_device::irq_smartcard_w(int state)
{
	spot_asic_device::set_bus_irq(BUS_INT_DEVSMC, state);
}

void spot_asic_device::irq_audio_w(int state)
{
	spot_asic_device::set_bus_irq(BUS_INT_AUDDMA, state);
}

void spot_asic_device::irq_modem_w(int state)
{
	spot_asic_device::set_bus_irq(BUS_INT_DEVMOD, state);
}

void spot_asic_device::set_bus_irq(uint8_t mask, int state)
{
	if (m_intenable & mask)
	{
		if (state)
			m_intstat |= mask;
		else
			m_intstat &= ~(mask);
		
		m_hostcpu->set_input_line(MIPS3_IRQ0, state ? ASSERT_LINE : CLEAR_LINE);
	}
}

void spot_asic_device::set_vid_irq(uint8_t mask, int state)
{
	if (m_vid_intenable & mask)
	{
		if (state)
			m_vid_intstat |= mask;
		else
			m_vid_intstat &= ~(mask);

		spot_asic_device::set_bus_irq(BUS_INT_VIDINT, state);
	}
}

uint32_t spot_asic_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	uint16_t screen_width = bitmap.width();
	uint16_t screen_height = bitmap.height();
	uint8_t vid_step = (2 * VID_BYTES_PER_PIXEL);

	m_vid_cstart = m_vid_nstart;
	m_vid_csize = m_vid_nsize;
	m_vid_ccnt = m_vid_draw_nstart;

	address_space &space = m_hostcpu->space(AS_PROGRAM);

	for (int y = 0; y < screen_height; y++)
	{
		uint32_t *line = &bitmap.pix(y);

		m_vid_cline = y;

		if (m_vid_cline == m_vid_hintline)
			spot_asic_device::set_vid_irq(VID_INT_HSYNC, 1);

		for (int x = 0; x < screen_width; x += 2)
		{
			uint32_t pixel = VID_DEFAULT_COLOR;

			bool is_active_area = (
				y >= m_vid_draw_vstart
				&& y < (m_vid_draw_vstart + m_vid_draw_vsize)

				&& x >= m_vid_draw_hstart
				&& x < (m_vid_draw_hstart + m_vid_draw_hsize)
			);

			if (m_vid_fcntl & VID_FCNTL_VIDENAB && m_vid_dmacntl & VID_DMACNTL_DMAEN && is_active_area)
			{
				pixel = space.read_dword(m_vid_ccnt);

				m_vid_ccnt += vid_step;
			}
			else if (m_vid_fcntl & VID_FCNTL_BLNKCOLEN)
			{
				pixel = m_vid_draw_blank_color;
			}

			int32_t y1 = ((pixel >> 0x18) & 0xff) - VID_Y_BLACK;
			int32_t Cb = ((pixel >> 0x10) & 0xff) - VID_UV_OFFSET;
			int32_t y2 = ((pixel >> 0x08) & 0xff) - VID_Y_BLACK;
			int32_t Cr = ((pixel) & 0xff) - VID_UV_OFFSET;

			y1 = (((y1 << 8) + VID_UV_OFFSET) / VID_Y_RANGE);
			y2 = (((y2 << 8) + VID_UV_OFFSET) / VID_Y_RANGE);

			int32_t r = ((0x166 * Cr) + VID_UV_OFFSET) >> 8;
			int32_t b = ((0x1C7 * Cb) + VID_UV_OFFSET) >> 8;
			int32_t g = ((0x32 * b) + (0x83 * r) + VID_UV_OFFSET) >> 8;

			*line++ = (
				std::clamp(y1 + r, 0x00, 0xff) << 0x10
				| std::clamp(y1 - g, 0x00, 0xff) << 0x08
				| std::clamp(y1 + b, 0x00, 0xff)
			);

			*line++ = (
				std::clamp(y2 + r, 0x00, 0xff) << 0x10
				| std::clamp(y2 - g, 0x00, 0xff) << 0x08
				| std::clamp(y2 + b, 0x00, 0xff)
			);
		}
	}

	spot_asic_device::set_vid_irq(VID_INT_DMA, 1);

	return 0;
}
