// license:BSD-3-Clause
// copyright-holders:FairPlay137

/***************************************************************************************************

    solo1_asic.cpp

    WebTV Networks Inc. SOLO1 ASIC

    This ASIC controls most of the I/O on the 2nd generation WebTV hardware.

    This implementation is based off of both the archived technical specifications, as well as
    the various reverse-engineering efforts of the WebTV community.

    The technical specifications that this implementation is based on can be found here:
    http://wiki.webtv.zone/misc/SOLO1/SOLO1_ASIC_Spec.pdf

    The SOLO ASIC is split into multiple "units", of which this implementation currently only
    emulates the busUnit, the memUnit, and the devUnit.

    The rioUnit (0xA4001xxx) provides a shared interface to the ROM, asynchronous devices (including
    the modem, the IDE hard drive, and the IDE CD-ROM), and synchronous devices which plug into the
    WebTV Port connector (which did not see much use other than for the FIDO/FCS printer interface).

    The audUnit (0xA4002xxx) handles audio DMA.

    The vidUnit (0xA4003xxx) handles video DMA.

    The devUnit (0xA4004xxx) handles GPIO, IR input, IR blaster output, front panel LEDs, and the
    parallel port.

    The memUnit (0xA4005xxx) handles memory timing and other memory-related operations. These
    registers are only emulated for completeness; they do not currently have an effect on the
    emulation.

    The gfxUnit (0xA4006xxx) is responsible for accelerated graphics.

    The dveUnit (0xA4007xxx) is responsible for digital video encoding.

    The divUnit (0xA4008xxx) is responsible for video input decoding.

    The potUnit (0xA4009xxx) handles low-level video output, previously handled by SPOT's vidUnit.

    The sucUnit (0xA400Axxx) handles serial I/O for both the RS232 port and the SmartCard reader.

    The modUnit (0xA400Bxxx) handles softmodem I/O. It's basically a stripped down audUnit. Whether
    this will be emulated is uncertain.

****************************************************************************************************/
#include "emu.h"

#include "solo1_asic.h"

#define LOG_UNKNOWN     (1U << 1)
#define LOG_READS       (1U << 2)
#define LOG_WRITES      (1U << 3)
#define LOG_ERRORS      (1U << 4)
#define LOG_I2C_IGNORES (1U << 5)
#define LOG_DEFAULT     (LOG_READS | LOG_WRITES | LOG_ERRORS | LOG_I2C_IGNORES | LOG_UNKNOWN)

#define LOG_BUSUNIT     (1U << 6)
#define LOG_RIOUNIT     (1U << 7)
#define LOG_AUDUNIT     (1U << 8)
#define LOG_VIDUNIT     (1U << 9)
#define LOG_DEVUNIT     (1U << 10)
#define LOG_MEMUNIT     (1U << 11)
#define LOG_GFXUNIT     (1U << 12)
#define LOG_DVEUNIT     (1U << 13)
#define LOG_DIVUNIT     (1U << 14)
#define LOG_POTUNIT     (1U << 15)
#define LOG_SUCUNIT     (1U << 16)
#define LOG_MODUNIT     (1U << 17)

#define VERBOSE         (LOG_DEFAULT)
#include "logmacro.h"

#define BUS_INT_VIDEO 1 << 7 // Video interrupt
#define BUS_INT_AUDIO 1 << 6 // Audio interrupt
#define BUS_INT_RIO   1 << 5 // RIO device interrupt
#define BUS_INT_SOLO1 1 << 4 // SOLO1 device interrupt
#define BUS_INT_TIMER 1 << 3 // Timer interrupt
#define BUS_INT_FENCE 1 << 2 // Fence (error) interrupt

#define BUS_GPINTSTAT_15 1 << 17
#define BUS_GPINTSTAT_14 1 << 16
#define BUS_GPINTSTAT_13 1 << 15
#define BUS_GPINTSTAT_12 1 << 14
#define BUS_GPINTSTAT_11 1 << 13
#define BUS_GPINTSTAT_10 1 << 12
#define BUS_GPINTSTAT_9  1 << 11
#define BUS_GPINTSTAT_8  1 << 10
#define BUS_GPINTSTAT_7  1 << 9
#define BUS_GPINTSTAT_6  1 << 8
#define BUS_GPINTSTAT_5  1 << 7
#define BUS_GPINTSTAT_4  1 << 6
#define BUS_GPINTSTAT_3  1 << 5
#define BUS_GPINTSTAT_2  1 << 4
#define BUS_GPINTSTAT_1  1 << 3
#define BUS_GPINTSTAT_0  1 << 2

#define BUS_AUD_INT_SOFTMODEM_DMA_IN  1 << 6 // modUnit DMA input interrupt
#define BUS_AUD_INT_SOFTMODEM_DMA_OUT 1 << 5 // modUnit DMA output interrupt
#define BUS_AUD_INT_DIV_AUDIO         1 << 4 // divUnit audio interrupt
#define BUS_AUD_INT_DMA_IN            1 << 3 // audUnit DMA input interrupt
#define BUS_AUD_INT_DMA_OUT           1 << 2 // audUnit DMA output interrupt

#define BUS_VID_INT_DIV 1 << 5 // Interrupt trigger in divUnit
#define BUS_VID_INT_GFX 1 << 4 // Interrupt trigger in gfxUnit
#define BUS_VID_INT_POT 1 << 3 // Interrupt trigger in potUnit
#define BUS_VID_INT_VID 1 << 2 // Interrupt trigger in vidUnit (likely finished interrupt)

#define BUS_DEV_INT_GPIO      1 << 7 // GPIO interrupt
#define BUS_DEV_INT_UART      1 << 6 // sucUnit UART interrupt (RS232)
#define BUS_DEV_INT_SMARTCARD 1 << 5 // sucUnit SmartCard interrupt
#define BUS_DEV_INT_PARALLEL  1 << 4 // devUnit Parallel interrupt
#define BUS_DEV_INT_IR_OUT    1 << 3 // devUnit IR output interrupt
#define BUS_DEV_INT_IR_IN     1 << 2 // devUnit IR input interrupt

#define BUS_RIO_INT_DEVICE3 1 << 5 // Device 3 interrupt
#define BUS_RIO_INT_DEVICE2 1 << 4 // Device 2 interrupt
#define BUS_RIO_INT_DEVICE1 1 << 3 // Device 1 interrupt (typically IDE controller)
#define BUS_RIO_INT_DEVICE0 1 << 2 // Device 0 interrupt (typically modem/ethernet)

#define BUS_TIM_INT_SYSTIMER    1 << 3 // System timer interrupt
#define BUS_TIM_INT_BUS_TIMEOUT 1 << 2 // Bus timeout interrupt

#define BUS_RESETCAUSE_SOFTWARE 1 << 2 // Software reset
#define BUS_RESETCAUSE_WATCHDOG 1 << 1 // Watchdog reset
#define BUS_RESETCAUSE_SWITCH   1 << 0 // Reset button pressed

#define VID_INT_DMA       1 << 2

#define POT_INT_VSYNCE 1 << 5 // Even field VSYNC
#define POT_INT_VSYNCO 1 << 4 // Odd field VSYNC
#define POT_INT_HSYNC  1 << 3 // HSYNC hits HINTLINE value
#define POT_INT_SHIFT  1 << 2

#define POT_CNTL_DVE_USE_GFX_444  1 << 11
#define POT_CNTL_DVE_CRCBSEL      1 << 10
#define POT_CNTL_DVE_HALF_SHIFT   1 << 9
#define POT_CNTL_HFIELD_LINE      1 << 8
#define POT_CNTL_VID_SYNC_EN      1 << 7
#define POT_CNTL_VID_DOUT_EN      1 << 6
#define POT_CNTL_HALF_SHIFT       1 << 5
#define POT_CNTL_INVERT_CRCB      1 << 4
#define POT_CNTL_USE_GFXUNIT      1 << 3
#define POT_CNTL_SOFT_RESET       1 << 2
#define POT_CNTL_PROGRESSIVE_SCAN 1 << 1
#define POT_CNTL_ENABLE_OUTPUTS   1 << 0

#define NTSC_SCREEN_XTAL   XTAL(18'432'000)
#define NTSC_SCREEN_WIDTH  640
#define NTSC_SCREEN_HSTART 40
#define NTSC_SCREEN_HSIZE  560
#define NTSC_SCREEN_HEIGHT 480
#define NTSC_SCREEN_VSTART 30
#define NTSC_SCREEN_VSIZE  420

#define VID_DEFAULT_XTAL   NTSC_SCREEN_XTAL
#define VID_DEFAULT_WIDTH  NTSC_SCREEN_WIDTH
#define VID_DEFAULT_HSTART NTSC_SCREEN_HSTART
#define VID_DEFAULT_HSIZE  NTSC_SCREEN_HSIZE
#define VID_DEFAULT_HEIGHT NTSC_SCREEN_HEIGHT
#define VID_DEFAULT_VSTART NTSC_SCREEN_VSTART
#define VID_DEFAULT_VSIZE  NTSC_SCREEN_VSIZE
// This is always 0x77 on SPOT and SOLO for some reason (even on hardware)
// This is needed to correct the HSTART value.
#define VID_HSTART_OFFSET  0x77

#define VID_Y_BLACK         0x10
#define VID_Y_WHITE         0xeb
#define VID_Y_RANGE         (VID_Y_WHITE - VID_Y_BLACK)
#define VID_UV_OFFSET       0x80
#define VID_BYTES_PER_PIXEL 2
#define VID_DEFAULT_COLOR   (VID_UV_OFFSET << 0x10) | (VID_Y_BLACK << 0x08) | VID_UV_OFFSET;

DEFINE_DEVICE_TYPE(SOLO1_ASIC, solo1_asic_device, "solo1_asic", "WebTV SOLO1 ASIC")

solo1_asic_device::solo1_asic_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SOLO1_ASIC, tag, owner, clock),
	device_serial_interface(mconfig, *this),
	device_video_interface(mconfig, *this),
    m_hostcpu(*this, finder_base::DUMMY_TAG),
	m_serial_id(*this, finder_base::DUMMY_TAG),
	m_nvram(*this, finder_base::DUMMY_TAG),
	m_screen(*this, "screen"),
	m_dac(*this, "dac%u", 0),
	m_lspeaker(*this, "lspeaker"),
	m_rspeaker(*this, "rspeaker"),
	m_watchdog(*this, "watchdog"),
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

void solo1_asic_device::bus_unit_map(address_map &map)
{
}

void solo1_asic_device::rio_unit_map(address_map &map)
{
}

void solo1_asic_device::aud_unit_map(address_map &map)
{
}

void solo1_asic_device::vid_unit_map(address_map &map)
{
}

void solo1_asic_device::dev_unit_map(address_map &map)
{
}

void solo1_asic_device::mem_unit_map(address_map &map)
{
}

void solo1_asic_device::gfx_unit_map(address_map &map)
{
}

void solo1_asic_device::dve_unit_map(address_map &map)
{
}

void solo1_asic_device::div_unit_map(address_map &map)
{
}

void solo1_asic_device::pot_unit_map(address_map &map)
{
}

void solo1_asic_device::suc_unit_map(address_map &map)
{
}

void solo1_asic_device::mod_unit_map(address_map &map)
{
}

uint32_t solo1_asic_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	uint16_t screen_width = bitmap.width();
	uint16_t screen_height =  bitmap.height();

	m_vid_cstart = m_vid_nstart;
	m_vid_csize = m_vid_nsize;
	m_vid_ccnt = m_pot_drawstart;

	address_space &space = m_hostcpu->space(AS_PROGRAM);

	for (int y = 0; y < screen_height; y++)
	{
		uint32_t *line = &bitmap.pix(y);

		m_pot_cline = y;

		if (m_pot_cline == m_pot_hintline)
			solo1_asic_device::set_pot_irq(POT_INT_HSYNC, 1);

		for (int x = 0; x < screen_width; x += 2)
		{
			int32_t pixel = VID_DEFAULT_COLOR;

			bool is_active_area = (
				y >= m_pot_vstart
				&& y < (m_pot_vstart + m_pot_drawvsize)

				&& x >= m_pot_hstart
				&& x < (m_pot_hstart + m_pot_hsize)
			);

			if (m_vid_fcntl & POT_CNTL_ENABLE_OUTPUTS && m_vid_dmacntl & VID_DMACNTL_DMAEN && is_active_area)
			{
				pixel = space.read_dword(m_vid_ccnt);

				m_vid_ccnt += 2 * VID_BYTES_PER_PIXEL;
			}
			else if (m_vid_fcntl & VID_FCNTL_BLNKCOLEN)
			{
				pixel = m_pot_blank_color | (((m_pot_blank_color >> 0x08) & 0xff) << 0x18);
			}

			int32_t y1 = ((pixel >> 0x18) & 0xff) - VID_Y_BLACK;
			int32_t Cb  = ((pixel >> 0x10) & 0xff) - VID_UV_OFFSET;
			int32_t y2 = ((pixel >> 0x08) & 0xff) - VID_Y_BLACK;
			int32_t Cr  = ((pixel) & 0xff) - VID_UV_OFFSET;

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

	solo1_asic_device::irq_vid_w(VID_INT_DMA, 1);

	return 0;
}

void solo1_asic_device::irq_aud_w(uint32_t value)
{
    m_bus_int_status |= BUS_INT_AUDIO;
    m_bus_aud_int_status |= value;
}

void solo1_asic_device::irq_vid_w(uint32_t value)
{
    m_bus_int_status |= BUS_INT_VIDEO;
    m_bus_vid_int_status |= value;
}

void solo1_asic_device::irq_rio_w(uint32_t value)
{
    m_bus_int_status |= BUS_INT_RIO;
    m_bus_rio_int_status |= value;
}

void solo1_asic_device::set_pot_irq(uint8_t mask, int state)
{
}

void solo1_asic_device::device_add_mconfig(machine_config &config)
{
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_screen_update(FUNC(solo1_asic_device::screen_update));
	m_screen->screen_vblank().set(FUNC(solo1_asic_device::vblank_irq));
	m_screen->set_raw(VID_DEFAULT_XTAL, VID_DEFAULT_WIDTH, 0, VID_DEFAULT_WIDTH, VID_DEFAULT_HEIGHT, 0, VID_DEFAULT_HEIGHT);

	SPEAKER(config, m_lspeaker).front_left();
	SPEAKER(config, m_rspeaker).front_right();
	DAC_16BIT_R2R_TWOS_COMPLEMENT(config, m_dac[0], 0).add_route(0, m_lspeaker, 0.0);
	DAC_16BIT_R2R_TWOS_COMPLEMENT(config, m_dac[1], 0).add_route(0, m_rspeaker, 0.0);
}

void solo1_asic_device::device_start()
{
    m_sys_timer = timer_alloc(FUNC(solo1_asic_device::sys_timer_callback), this);
    m_bus_chip_id = 0x03120000; // SOLO1 chip ID - this should not change

    save_item(NAME(m_bus_chip_cntl));
    save_item(NAME(m_bus_int_status));
    save_item(NAME(m_bus_int_enable));
    save_item(NAME(m_bus_err_status));
    save_item(NAME(m_bus_err_enable));
    save_item(NAME(m_bus_err_address));
    save_item(NAME(m_bus_wd_reset_val));
    save_item(NAME(m_bus_lomem_rdprot_addr));
    save_item(NAME(m_bus_lomem_rdprot_mask));
    save_item(NAME(m_bus_lomem_wrprot_addr));
    save_item(NAME(m_bus_lomem_wrprot_mask));
    save_item(NAME(m_bus_tmr_compare));
    //save_item(NAME(m_compare_armed));
    save_item(NAME(m_bus_gpio_int_status));
    save_item(NAME(m_bus_gpio_int_enable));
    save_item(NAME(m_bus_gpio_int_polling));
    save_item(NAME(m_bus_aud_int_status));
    save_item(NAME(m_bus_aud_int_enable));
    save_item(NAME(m_bus_dev_int_status));
    save_item(NAME(m_bus_dev_int_enable));
    save_item(NAME(m_bus_vid_int_status));
    save_item(NAME(m_bus_vid_int_enable));
    save_item(NAME(m_bus_rio_int_status));
    save_item(NAME(m_bus_rio_int_enable));
    save_item(NAME(m_bus_rio_int_polling));
    save_item(NAME(m_bus_tim_int_status));
    save_item(NAME(m_bus_tim_int_enable));
    save_item(NAME(m_bus_reset_cause));
    save_item(NAME(m_bus_java1_fence_addr_l));
    save_item(NAME(m_bus_java1_fence_addr_h));
    save_item(NAME(m_bus_java2_fence_addr_l));
    save_item(NAME(m_bus_java2_fence_addr_h));
    save_item(NAME(m_bus_memsize));
    save_item(NAME(m_bus_fence_cntl));
    save_item(NAME(m_bus_bootmode));
    save_item(NAME(m_bus_use_bootmode));
}

void solo1_asic_device::device_reset()
{
    m_sys_timer->adjust(attotime::never); // disable

    m_bus_chip_cntl = 0;

    m_bus_int_status = 0;
    m_bus_int_enable = 0;

    m_bus_aud_int_enable = 0;
    m_bus_aud_int_status = 0;
    
    m_bus_vid_int_enable = 0;
    m_bus_vid_int_status = 0;
    
    m_bus_rio_int_enable = 0;
    m_bus_rio_int_status = 0;

    m_bus_tim_int_enable = 0;
    m_bus_tim_int_status = 0;

    m_bus_java1_fence_addr_l = 0;
    m_bus_java1_fence_addr_h = 0;
    m_bus_java2_fence_addr_l = 0;
    m_bus_java2_fence_addr_h = 0;

    m_bus_tmr_count = 0;

    m_bus_memsize = 0x04000000;

    m_bus_reset_cause = BUS_RESETCAUSE_SWITCH; // reset button hit
}

void solo1_asic_device::solo1_update_cycle_counting()
{
    m_bus_tmr_count = m_clock;
    if(m_compare_armed)
    {
        uint32_t delta = m_bus_tmr_compare - m_bus_tmr_count;
        m_sys_timer->adjust(clocks_to_attotime(delta));
    }
}

uint32_t solo1_asic_device::reg_0000_r()
{
    return m_bus_chip_id;
}

uint32_t solo1_asic_device::reg_0004_r()
{
    return 0;
}

void solo1_asic_device::reg_0004_w(uint32_t data)
{
}

TIMER_CALLBACK_MEMBER(solo1_asic_device::sys_timer_callback)
{
    m_sys_timer->adjust(attotime::never);
    m_compare_armed = 0;
    m_bus_tmr_count = m_bus_tmr_compare;
    if((m_bus_int_enable & BUS_INTSTAT_TIMER)&&(m_bus_tim_int_enable & BUS_TIM_INTSTAT_SYSTIMER))
    {
        m_bus_int_status |= BUS_INTSTAT_TIMER;
        m_bus_tim_int_status |= BUS_TIM_INTSTAT_SYSTIMER;
        m_hostcpu->set_input_line(MIPS3_IRQ0, ASSERT_LINE);
    }
}

uint32_t solo1_asic_device::reg_bus_r(offs_t offset)
{
    // TODO: split this out into multiple handlers! using a giant switch statement for this is just ugly
    LOGMASKED(LOG_READS, "busUnit: read %04x\n", offset * 4);
    switch(offset * 4)
    {
    case 0x000: // BUS_CHIPID (R/W)
        return m_bus_chip_id;
    case 0x004: // BUS_CHIPCNTL (R/W)
        return m_bus_chip_cntl;
    case 0x008: // BUS_INTSTAT (R/W)
        return m_bus_int_status & m_bus_int_enable; // TODO: is this correct behavior?
    case 0x00c: // BUS_INTEN (R/Set)
        return m_bus_int_enable;
    case 0x010: // BUS_ERRSTAT (W)
        break;
    case 0x110: // BUS_ERRSTAT (Clear)
        break;
    case 0x014: // BUS_ERREN (R/Set)
        return m_bus_err_enable;
    case 0x114: // BUS_ERREN (Clear)
        break;
    case 0x018: // BUS_ERRADDR (R/W)
        return m_bus_err_address;
    case 0x030: // BUS_WDVALUE (R/W)
        return m_bus_wd_reset_val;
    case 0x034: // BUS_LOWRDADDR (R/W)
        return m_bus_lomem_rdprot_addr;
    case 0x038: // BUS_LOWRDMASK (R/W)
        return m_bus_lomem_rdprot_mask;
    case 0x03c: // BUS_LOWWRADDR (R/W)
        return m_bus_lomem_wrprot_addr;
    case 0x040: // BUS_LOWWRMASK (R/W)
        return m_bus_lomem_wrprot_mask;
    case 0x048: // BUS_TCOUNT (R/W)
        m_bus_tmr_count = m_clock;
        return m_bus_tmr_count;
    case 0x04c: // BUS_TCOMPARE (R/W)
        return m_bus_tmr_compare;
    case 0x050: // BUS_INTSTAT (Set)
        break;
    case 0x054: // BUS_ERRSTAT (R/Set)
        return m_bus_err_status;
    case 0x058: // BUS_GPINTSTAT (W)
        break;
    case 0x05c: // BUS_GPINTEN (R/Set)
        return m_bus_gpio_int_enable;
    case 0x15c: // BUS_GPINTEN (Clear)
        break;
    case 0x060: // BUS_GPINTSTAT (R/Set)
        return m_bus_gpio_int_status & m_bus_gpio_int_enable; // TODO: is this correct behavior?
    case 0x064: // BUS_GPINTPOL (R/W)
        return m_bus_gpio_int_polling;
    case 0x068: // BUS_AUDINTSTAT (W)
        break;
    case 0x168: // BUS_AUDINTSTAT (Clear)
        break;
    case 0x06c: // BUS_AUDINTSTAT (R/Set)
        return m_bus_aud_int_status & m_bus_aud_int_enable; // TODO: is this correct behavior?
    case 0x070: // BUS_AUDINTEN (R/Set)
        return m_bus_aud_int_enable;
    case 0x170: // BUS_AUDINTEN (Clear)
        break;
    case 0x074: // BUS_DEVINTSTAT (W)
        break;
    case 0x174: // BUS_DEVINTSTAT (Clear)
        break;
    case 0x078: // BUS_DEVINTSTAT (R/Set)
        return m_bus_dev_int_status & m_bus_dev_int_enable; // TODO: is this correct behavior?
    case 0x07c: // BUS_DEVINTEN (R/Set)
        return m_bus_dev_int_enable;
    case 0x17c: // BUS_DEVINTEN (Clear)
        break;
    case 0x080: // BUS_VIDINTSTAT (W)
        break;
    case 0x180: // BUS_VIDINTSTAT (Clear)
        break;
    case 0x084: // BUS_VIDINTSTAT (R/Set)
        return m_bus_vid_int_status & m_bus_vid_int_enable; // TODO: is this correct behavior?
    case 0x088: // BUS_VIDINTEN (R/Set)
        return m_bus_vid_int_enable;
    case 0x188: // BUS_VIDINTEN (Clear)
        break;
    case 0x08c: // BUS_RIOINTSTAT (W)
        break;
    case 0x18c: // BUS_RIOINTSTAT (Clear)
        break;
    case 0x094: // BUS_RIOINTPOL (R/W)
        return m_bus_rio_int_polling;
    case 0x090: // BUS_RIOINTSTAT (R/Set)
        return m_bus_rio_int_status & m_bus_rio_int_enable; // TODO: is this correct behavior?
    case 0x098: // BUS_DEVINTEN (R/Set)
        return m_bus_rio_int_enable;
    case 0x198: // BUS_DEVINTEN (Clear)
        break;
    case 0x09c: // BUS_TIMINTSTAT (W)
        break;
    case 0x19c: // BUS_TIMINTSTAT (Clear)
        break;
    case 0x0a0: // BUS_TIMINTSTAT (R/Set)
        return m_bus_tim_int_status & m_bus_tim_int_enable; // TODO: is this correct behavior?
    case 0x0a4: // BUS_TIMINTEN (R/Set)
        return m_bus_tim_int_enable;
    case 0x1a4: // BUS_TIMINTEN (Clear)
        break;
    case 0x0a8: // RESETCAUSE (R/Set)
        return m_bus_reset_cause;
    case 0x0ac: // RESETCAUSE (Clear)
        break;
    case 0x0b0: // BUS_J1FENLADDR (R/W)
        return m_bus_java1_fence_addr_l;
    case 0x0b4: // BUS_J1FENHADDR (R/W)
        return m_bus_java1_fence_addr_h;
    case 0x0b8: // BUS_J2FENLADDR (R/W)
        return m_bus_java2_fence_addr_l;
    case 0x0bc: // BUS_J2FENHADDR (R/W)
        return m_bus_java2_fence_addr_h;
    case 0x0c0: // BUS_TOPOFRAM (R/W)
        return m_bus_memsize;
    case 0x0c4: // BUS_FENCECNTL (R/W)
        return m_bus_fence_cntl;
    case 0x0c8: // BUS_BOOTMODE (R/W)
        return m_bus_bootmode;
    case 0x0cc: // BUS_USEBOOTMODE (R/W)
        return m_bus_use_bootmode;
    default:
        logerror("Attempted read from reserved register 0%03x!\n", offset * 4);
        break;
    }
    return 0;
}

void solo1_asic_device::reg_bus_w(offs_t offset, uint32_t data)
{
    // TODO: split this out into multiple handlers! using a giant switch statement for this is just ugly
    LOGMASKED(LOG_WRITES, "busUnit: write %08x to %04x\n", data, offset * 4);
    switch(offset * 4)
    {
    case 0x000: // BUS_CHIPID (R/W)
        logerror("Attempted write (%08x) to read-only register 0000 (BUS_CHIPID)\n", data); // Presumed behavior, there is no real need to write to this register
        break;
    case 0x004: // BUS_CHIPCNTL (R/W)
        m_bus_chip_cntl = data;
        break;
    case 0x008: // BUS_INTSTAT (R/W)
        m_bus_int_status = data;
        break;
    case 0x108: // BUS_INTSTAT (Clear)
        m_bus_int_status &= ~data; // TODO: is this correct behavior?
        break;
    case 0x00c: // BUS_INTEN (R/Set)
        m_bus_int_enable |= data; // TODO: is this correct behavior?
        break;
    case 0x010: // BUS_ERRSTAT (W)
        m_bus_err_status = data;
        break;
    case 0x110: // BUS_ERRSTAT (Clear)
        m_bus_err_status &= ~data;
        break;
    case 0x014: // BUS_ERREN (R/Set)
        m_bus_err_enable |= data;
        break;
    case 0x114: // BUS_ERREN (Clear)
        m_bus_err_enable &= ~data;
        break;
    case 0x018: // BUS_ERRADDR (R/W)
        m_bus_err_enable = data;
        break;
    case 0x030: // BUS_WDVALUE (R/W)
        m_bus_wd_reset_val = data;
        break;
    case 0x034: // BUS_LOWRDADDR (R/W)
        m_bus_lomem_rdprot_addr = data;
        break;
    case 0x038: // BUS_LOWRDMASK (R/W)
        m_bus_lomem_rdprot_mask = data;
        break;
    case 0x03c: // BUS_LOWWRADDR (R/W)
        m_bus_lomem_wrprot_addr = data;
        break;
    case 0x040: // BUS_LOWWRMASK (R/W)
        m_bus_lomem_wrprot_mask = data;
        break;
    case 0x048: // BUS_TCOUNT (R/W)
        m_bus_tmr_count = data;
        solo1_update_cycle_counting();
        break;
    case 0x04c: // BUS_TCOMPARE (R/W)
        m_bus_tmr_compare = data;
        m_compare_armed = 1;
        solo1_update_cycle_counting();
        break;
    case 0x050: // BUS_INTSTAT (Set)
        m_bus_int_status |= data; // TODO: is this correct behavior?
        break;
    case 0x054: // BUS_ERRSTAT (R/Set)
        m_bus_err_status |= data; // TODO: is this correct behavior?
        break;
    case 0x058: // BUS_GPINTSTAT (W)
        m_bus_gpio_int_status = data;
        break;
    case 0x158: // BUS_GPINTSTAT (Clear)
        m_bus_gpio_int_status &= ~data; // TODO: is this correct behavior?
        break;
    case 0x05c: // BUS_GPINTEN (R/Set)
        m_bus_gpio_int_enable |= data; // TODO: is this correct behavior?
        break;
    case 0x15c: // BUS_GPINTEN (Clear)
        m_bus_gpio_int_enable &= ~data; // TODO: is this correct behavior?
        break;
    case 0x060: // BUS_GPINTSTAT (W)
        m_bus_gpio_int_status = data;
        break;
    case 0x064: // BUS_GPINTPOL (R/W)
        m_bus_gpio_int_polling = data;
        break;
    case 0x068: // BUS_AUDINTSTAT (W)
        m_bus_aud_int_status = data; // TODO: is this correct behavior? also mirror this over to audUnit (solo1_asic_aud)
        break;
    case 0x168: // BUS_AUDINTSTAT (Clear)
        m_bus_aud_int_status &= ~data; // TODO: is this correct behavior? also mirror this over to audUnit (solo1_asic_aud)
        break;
    case 0x06c: // BUS_AUDINTSTAT (R/Set)
        m_bus_aud_int_status |= data; // TODO: is this correct behavior? also mirror this over to audUnit (solo1_asic_aud)
        break;
    case 0x070: // BUS_AUDINTEN (R/Set)
        m_bus_aud_int_enable |= data; // TODO: is this correct behavior? also mirror this over to audUnit (solo1_asic_aud)
        break;
    case 0x170: // BUS_AUDINTEN (Clear)
        m_bus_aud_int_enable &= ~data; // TODO: is this correct behavior? also mirror this over to audUnit (solo1_asic_aud)
        break;
    case 0x074: // BUS_DEVINTSTAT (W)
        m_bus_dev_int_status = data; // TODO: is this correct behavior? also mirror this over to devUnit
        break;
    case 0x174: // BUS_DEVINTSTAT (Clear)
        m_bus_dev_int_status &= ~data; // TODO: is this correct behavior? also mirror this over to devUnit
        break;
    case 0x078: // BUS_DEVINTSTAT (R/Set)
        m_bus_dev_int_status |= data; // TODO: is this correct behavior? also mirror this over to devUnit
        break;
    case 0x07c: // BUS_DEVINTEN (R/Set)
        m_bus_dev_int_enable |= data; // TODO: is this correct behavior? also mirror this over to devUnit
        break;
    case 0x17c: // BUS_DEVINTEN (Clear)
        m_bus_dev_int_enable &= ~data; // TODO: is this correct behavior? also mirror this over to devUnit
        break;
    case 0x080: // BUS_VIDINTSTAT (W)
        m_bus_vid_int_status = data; // TODO: is this correct behavior? also mirror this over to vidUnit (solo1_asic_vid)
        break;
    case 0x180: // BUS_VIDINTSTAT (Clear)
        m_bus_vid_int_status &= ~data; // TODO: is this correct behavior? also mirror this over to vidUnit (solo1_asic_vid)
        break;
    case 0x084: // BUS_VIDINTSTAT (R/Set)
        m_bus_vid_int_status |= data; // TODO: is this correct behavior? also mirror this over to vidUnit (solo1_asic_vid)
        break;
    case 0x088: // BUS_VIDINTEN (R/Set)
        m_bus_vid_int_enable |= data; // TODO: is this correct behavior? also mirror this over to vidUnit (solo1_asic_vid)
        break;
    case 0x188: // BUS_VIDINTEN (Clear)
        m_bus_vid_int_enable &= ~data; // TODO: is this correct behavior? also mirror this over to vidUnit (solo1_asic_vid)
        break;
    case 0x08c: // BUS_RIOINTSTAT (W)
        m_bus_rio_int_status = data; // TODO: is this correct behavior? also mirror this over to rioUnit
        break;
    case 0x18c: // BUS_RIOINTSTAT (Clear)
        m_bus_rio_int_status &= ~data; // TODO: is this correct behavior? also mirror this over to rioUnit
        break;
    case 0x090: // BUS_RIOINTSTAT (R/Set)
        m_bus_rio_int_status |= data; // TODO: is this correct behavior? also mirror this over to rioUnit
        break;
    case 0x094: // BUS_RIOINTPOL (R/W)
        m_bus_rio_int_polling = data; // TODO: mirror this over to rioUnit
        break;
    case 0x098: // BUS_RIOINTEN (R/Set)
        m_bus_rio_int_enable |= data; // TODO: is this correct behavior? also mirror this over to rioUnit
        break;
    case 0x198: // BUS_RIOINTEN (Clear)
        m_bus_rio_int_enable &= ~data; // TODO: is this correct behavior? also mirror this over to rioUnit
        break;
    case 0x09c: // BUS_TIMINTSTAT (W)
        m_bus_tim_int_status = data;
        break;
    case 0x19c: // BUS_TIMINTSTAT (Clear)
        m_bus_tim_int_status &= ~data; // TODO: is this correct behavior?
        break;
    case 0x0a0: // BUS_TIMINTSTAT (R/Set)
        m_bus_tim_int_status |= data; // TODO: is this correct behavior?
        break;
    case 0x0a4: // BUS_TIMINTEN (R/Set)
        m_bus_tim_int_enable |= data; // TODO: is this correct behavior?
        break;
    case 0x1a4: // BUS_TIMINTEN (Clear)
        m_bus_tim_int_enable &= ~data; // TODO: is this correct behavior?
        break;
    case 0x0a8: // RESETCAUSE (R/Set)
        m_bus_reset_cause |= data; // TODO: is this correct behavior?
        break;
    case 0x0ac: // RESETCAUSE (Clear)
        m_bus_reset_cause &= ~data; // TODO: is this correct behavior?
        break;
    case 0x0b0: // BUS_J1FENLADDR (R/W)
        m_bus_java1_fence_addr_l = data;
        break;
    case 0x0b4: // BUS_J1FENHADDR (R/W)
        m_bus_java1_fence_addr_h = data;
        break;
    case 0x0b8: // BUS_J2FENLADDR (R/W)
        m_bus_java2_fence_addr_l = data;
        break;
    case 0x0bc: // BUS_J2FENHADDR (R/W)
        m_bus_java2_fence_addr_h = data;
        break;
    case 0x0c0: // BUS_TOPOFRAM (R/W)
        m_bus_memsize = data;
        break;
    case 0x0c4: // BUS_FENCECNTL (R/W)
        m_bus_fence_cntl = data;
        break;
    case 0x0c8: // BUS_BOOTMODE (R/W)
        m_bus_bootmode = data;
        break;
    case 0x0cc: // BUS_USEBOOTMODE (R/W)
        m_bus_use_bootmode = data;
        break;
    default:
        logerror("Attempted write (%08x) to reserved register 0%03x!\n", data, offset*4);
        break;
    }
}

uint32_t solo1_asic_device::reg_dev_r(offs_t offset)
{
    // TODO: split this out into multiple handlers! using a giant switch statement for this is just ugly
    LOGMASKED(LOG_READS, "devUnit: read 4%03x\n", (offset+2)*4);
    switch((offset+2) * 4)
    {
    case 0x000: // DEV_IROLD (R/W)
        // TODO: Remove this case! This isn't used anymore!
        return m_dev_irold;
    case 0x004: // DEV_LED (R/W)
        // TODO: Remove this case! This isn't used anymore!
        return m_dev_led;
    case 0x008: // DEV_IDCNTL (R/W)
        return m_dev_id_chip_cntl;
    case 0x00c: // DEV_IICCNTL (R/W)
        return m_dev_iic_cntl;
    case 0x010: // DEV_GPIOIN (R/W)
        return m_dev_gpio_in;
    case 0x014: // DEV_GPIOOUT (R/Set)
        return m_dev_gpio_out;
    case 0x114: // DEV_GPIOOUT (Clear)
        break;
    case 0x018: // DEV_GPIOEN (R/Set)
        return m_dev_gpio_en;
    case 0x118: // DEV_GPIOEN (Clear)
        break;
    case 0x020: // DEV_IRIN_SAMPLE_INT (R/W)
        return m_dev_ir_in_sample_int;
    case 0x024: // DEV_IRIN_REJECT_INT (R/W)
        return m_dev_ir_in_reject_int;
    case 0x028: // DEV_IRIN_TRANS_DATA (R/W)
        return m_dev_ir_in_transition_data;
    case 0x02c: // DEV_IRIN_STATCNTL (R/W)
        return m_dev_ir_in_status_cntl;
    case 0x040: // DEV_IROUT_FIFO (R/W)
        return m_dev_ir_out_fifo;
    case 0x044: // DEV_IROUT_STATUS (R/W)
        return m_dev_ir_out_status;
    case 0x048: // DEV_IROUT_PERIOD (R/W)
        return m_dev_ir_out_period;
    case 0x04c: // DEV_IROUT_ON (R/W)
        return m_dev_ir_out_on;
    case 0x050: // DEV_IROUT_CURRENT_PERIOD (R/W)
        return m_dev_ir_out_current_period;
    case 0x054: // DEV_IROUT_CURRENT_ON (R/W)
        return m_dev_ir_out_current_on;
    case 0x058: // DEV_IROUT_CURRENT_COUNT (R/W)
        return m_dev_ir_out_current_count;
    case 0x200: // DEV_PPORT_DATA (R/W)
        return m_dev_parallel_data;
    case 0x204: // DEV_PPORT_CTRL (R/W)
        return m_dev_parallel_ctrl;
    case 0x208: // DEV_PPORT_STAT (R/W)
        return m_dev_parallel_status;
    case 0x20c: // DEV_PPORT_CNFG (R/W)
        return m_dev_parallel_cnfg;
    case 0x210: // DEV_PPORT_FIFOCTRL (R/W)
        return m_dev_parallel_fifo_ctrl;
    case 0x214: // DEV_PPORT_FIFOSTAT (R/W)
        return m_dev_parallel_fifo_status;
    case 0x218: // DEV_PPORT_TIMEOUT (R/W)
        return m_dev_parallel_timeout;
    case 0x21c: // DEV_PPORT_STAT2 (R/W)
        return m_dev_parallel_stat2;
    case 0x220: // DEV_PPORT_IEN (R/W)
        return m_dev_parallel_int_enable;
    case 0x224: // DEV_PPORT_IEN (R/W)
        return m_dev_parallel_int_status;
    case 0x228: // DEV_PPORT_CLRINT (R/W)
        return m_dev_parallel_clr_int;
    case 0x22c: // DEV_PPORT_ENABLE (R/W)
        return m_dev_parallel_enable;
    case 0x804: // DEV_DIAG (R/W)
        return m_dev_diag;
    case 0x808: // DEV_DEVDIAG (R/W)
        return m_dev_devdiag;
    default:
        logerror("Attempted read from reserved register 4%03x!\n", offset*4);
        break;
    }
    return 0;
}

uint32_t solo1_asic_device::reg_dev_irold_r(offs_t offset)
{
    return m_dev_irold;
}

void solo1_asic_device::reg_dev_irold_w(offs_t offset, uint32_t data)
{
    m_dev_irold = data;
}

void solo1_asic_device::reg_dev_w(offs_t offset, uint32_t data)
{
    // TODO: split this out into multiple handlers! using a giant switch statement for this is just ugly
    LOGMASKED(LOG_WRITES, "devUnit: write %08x to 4%03x\n", data, (offset+2)*4);
    switch((offset+2)*4)
    {
    case 0x000: // DEV_IROLD (R/W)
        // TODO: Remove this case! This isn't used anymore!
        m_dev_irold = data;
        break;
    case 0x004: // DEV_LED (R/W)
        // TODO: Remove this case! This isn't used anymore!
        m_dev_led = data;
        popmessage("[DEBUG] LEDs: %d %d %d", (~data)&0x4/0x4, (~data)&0x2/0x2, (~data)&0x1);
        break;
    case 0x008: // DEV_IDCNTL (R/W)
        m_dev_id_chip_cntl = data;
        break;
    case 0x00c: // DEV_IICCNTL (R/W)
        // TODO: Implement the I2C bus
        m_dev_iic_cntl = data;
        break;
    case 0x010: // DEV_GPIOIN (R/W)
        m_dev_gpio_in = data;
        break;
    case 0x014: // DEV_GPIOOUT (R/Set)
        m_dev_gpio_out |= data;
        break;
    case 0x114: // DEV_GPIOOUT (Clear)
        m_dev_gpio_out &= ~data;
        break;
    case 0x018: // DEV_GPIOEN (R/Set)
        m_dev_gpio_en |= data;
        break;
    case 0x118: // DEV_GPIOEN (Clear)
        m_dev_gpio_en &= ~data;
        break;
    case 0x020: // DEV_IRIN_SAMPLE_INT (R/W)
        m_dev_ir_in_sample_int = data;
        break;
    case 0x024: // DEV_IRIN_REJECT_INT (R/W)
        m_dev_ir_in_reject_int = data;
        break;
    case 0x028: // DEV_IRIN_TRANS_DATA (R/W)
        m_dev_ir_in_transition_data = data;
        break;
    case 0x02c: // DEV_IRIN_STATCNTL (R/W)
        m_dev_ir_in_status_cntl = data;
        break;
    case 0x040: // DEV_IROUT_FIFO (R/W)
        m_dev_ir_out_fifo = data;
        break;
    case 0x044: // DEV_IROUT_STATUS (R/W)
        m_dev_ir_out_status = data;
        break;
    case 0x048: // DEV_IROUT_PERIOD (R/W)
        m_dev_ir_out_period = data;
        break;
    case 0x04c: // DEV_IROUT_ON (R/W)
        m_dev_ir_out_on = data;
        break;
    case 0x050: // DEV_IROUT_CURRENT_PERIOD (R/W)
        m_dev_ir_out_current_period = data;
        break;
    case 0x054: // DEV_IROUT_CURRENT_ON (R/W)
        m_dev_ir_out_current_on = data;
        break;
    case 0x058: // DEV_IROUT_CURRENT_COUNT (R/W)
        m_dev_ir_out_current_count = data;
        break;
    case 0x200: // DEV_PPORT_DATA (R/W)
        m_dev_parallel_data = data;
        break;
    case 0x204: // DEV_PPORT_CTRL (R/W)
        m_dev_parallel_ctrl = data;
        break;
    case 0x208: // DEV_PPORT_STAT (R/W)
        m_dev_parallel_status = data;
        break;
    case 0x20c: // DEV_PPORT_CNFG (R/W)
        m_dev_parallel_cnfg = data;
        break;
    case 0x210: // DEV_PPORT_FIFOCTRL (R/W)
        m_dev_parallel_fifo_ctrl = data;
        break;
    case 0x214: // DEV_PPORT_FIFOSTAT (R/W)
        m_dev_parallel_fifo_status = data;
        break;
    case 0x218: // DEV_PPORT_TIMEOUT (R/W)
        m_dev_parallel_timeout = data;
        break;
    case 0x21c: // DEV_PPORT_STAT2 (R/W)
        m_dev_parallel_stat2 = data;
        break;
    case 0x220: // DEV_PPORT_IEN (R/W)
        m_dev_parallel_int_enable = data;
        break;
    case 0x224: // DEV_PPORT_IST (R/W)
        m_dev_parallel_int_status = data;
        break;
    case 0x228: // DEV_PPORT_CLRINT (R/W)
        m_dev_parallel_clr_int = data;
        break;
    case 0x22c: // DEV_PPORT_ENABLE (R/W)
        m_dev_parallel_enable = data;
        break;
    case 0x804: // DEV_DIAG (R/W)
        m_dev_diag = data;
        break;
    case 0x808: // DEV_DEVDIAG (R/W)
        m_dev_devdiag = data;
        break;
    default:
        logerror("Attempted write (%08x) to reserved register 4%03x!\n", data, offset*4);
        break;
    }
}

uint32_t solo1_asic_device::reg_mem_r(offs_t offset)
{
    // TODO: split this out into multiple handlers! using a giant switch statement for this is just ugly
    LOGMASKED(LOG_READS, "memUnit: read 5%03x\n", offset*4);
    switch(offset*4)
    {
    case 0x000: // MEM_TIMING (R/W)
        return m_mem_timing;
    case 0x004: // MEM_CNTL (R/W)
        return m_mem_cntl;
    case 0x008: // MEM_BURP (R/W)
        return m_mem_burp;
    case 0x00c: // MEM_REFCNTL (R/W)
        return m_mem_refresh_cntl;
    case 0x010: // MEM_CMD (R/W)
        return m_mem_cmd;
    default:
        logerror("Attempted read from reserved register 5%03x!\n", offset*4);
        break;
    }
    return 0;
}

void solo1_asic_device::reg_mem_w(offs_t offset, uint32_t data)
{
    // TODO: split this out into multiple handlers! using a giant switch statement for this is just ugly
    LOGMASKED(LOG_WRITES, "memUnit: write %08x to 5%03x", data, offset*4);
    switch(offset*4)
    {
    case 0x000: // MEM_TIMING (R/W)
        m_mem_timing = data;
        break;
    case 0x004: // MEM_CNTL (R/W)
        m_mem_cntl = data;
        break;
    case 0x008: // MEM_BURP (R/W)
        m_mem_burp = data;
        break;
    case 0x00c: // MEM_REFCNTL (R/W)
        m_mem_refresh_cntl = data;
        break;
    case 0x010: // MEM_CMD (R/W)
        m_mem_cmd = data;
        break;
    default:
        logerror("Attempted write (%08x) to reserved register 5%03x!\n", data, offset*4);
        break;
    }
}

TIMER_CALLBACK_MEMBER(solo1_asic_device::dac_update)
{
	if(m_aud_dmacntl & AUD_DMACNTL_DMAEN)
	{
		if (m_aud_dma_ongoing)
		{
			address_space &space = m_hostcpu->space(AS_PROGRAM);

			int16_t samplel = space.read_dword(m_aud_ccnt);
			m_aud_ccnt += 2;
			int16_t sampler = space.read_dword(m_aud_ccnt);
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
			if(m_aud_ccnt >= m_aud_cend)
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