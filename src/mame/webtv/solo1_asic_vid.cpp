// license:BSD-3-Clause
// copyright-holders:FairPlay137

/***********************************************************************************************

    solo1_asic_vid.cpp

    WebTV Networks Inc. SOLO1 ASIC (Video)

    This ASIC controls most of the I/O on the 2nd generation WebTV hardware.

    This sub-device contains the potUnit and the vidUnit.

    The potUnit (Pixel Output Unit) controls the actual video signal going to the television.
    This implementation currently assumes a NTSC signal, but a PAL signal should be able to be
    emulated as well.

    The vidUnit controls the framebuffer being passed into the potUnit for display output.

    This implementation is based off of both the archived technical specifications, as well as
    the various reverse-engineering efforts of the WebTV community.

    The technical specifications that this implementation is based on can be found here:
    http://wiki.webtv.zone/misc/SOLO1/SOLO1_ASIC_Spec.pdf

    NOTE: This functionality is currently being handled within the main solo1_asic device. This
    device may serve as the video handler in the future once solo1_asic gets too complex, but
    for now this device is UNUSED.

************************************************************************************************/

/*
#include "emu.h"
#include "render.h"
#include "solo1_asic_vid.h"
#include "screen.h"

#define LOG_UNKNOWN     (1U << 1)
#define LOG_READS       (1U << 2)
#define LOG_WRITES      (1U << 3)
#define LOG_ERRORS      (1U << 4)
#define LOG_I2C_IGNORES (1U << 5)
#define LOG_DEFAULT     (LOG_READS | LOG_WRITES | LOG_ERRORS | LOG_I2C_IGNORES | LOG_UNKNOWN)

#define VERBOSE         (LOG_DEFAULT)
#include "logmacro.h"

#define VID_INT_DMA       1 << 2

#define SOLO1_NTSC_WIDTH 640
#define SOLO1_NTSC_HEIGHT 480

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

DEFINE_DEVICE_TYPE(SOLO1_ASIC_VID, solo1_asic_vid_device, "solo1_asic_vid", "WebTV SOLO1 ASIC (Video)")

solo1_asic_vid_device::solo1_asic_vid_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SOLO1_ASIC_VID, tag, owner, clock),
	device_video_interface(mconfig, *this),
    m_hostcpu(*this, finder_base::DUMMY_TAG),
    //m_soloasic(*this, finder_base::DUMMY_TAG),
	m_screen(*this, "screen"),
    m_videobitmap(SOLO1_NTSC_WIDTH, SOLO1_NTSC_HEIGHT),
    m_hsync_cb(*this),
	m_vsync_cb(*this)
{
    //m_soloasic = dynamic_cast<solo1_asic_device *>(owner);
}

void solo1_asic_vid_device::fillbitmap_yuy16(bitmap_yuy16 &bitmap, uint8_t yval, uint8_t cr, uint8_t cb)
{
	uint16_t color0 = (yval << 8) | cb;
	uint16_t color1 = (yval << 8) | cr;

	// write 32 bits of color (2 pixels at a time)
	for (int y = 0; y < bitmap.height(); y++)
	{
		uint16_t *dest = &bitmap.pix(y);
		for (int x = 0; x < bitmap.width() / 2; x++)
		{
			*dest++ = color0;
			*dest++ = color1;
		}
	}
}

void solo1_asic_vid_device::device_add_mconfig(machine_config &config)
{
    SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
    m_screen->set_size(SOLO1_NTSC_WIDTH, SOLO1_NTSC_HEIGHT);
	m_screen->set_visarea(0, SOLO1_NTSC_WIDTH - 1, 0, SOLO1_NTSC_HEIGHT - 1);
	m_screen->set_refresh_hz(59.94);
    
    m_screen->set_screen_update(FUNC(solo1_asic_vid_device::screen_update));
    set_clock(m_screen->clock() * 2); // internal clock is always set to double the pixel clock
}

void solo1_asic_vid_device::device_start()
{
    //m_screen->set_clock(3.579575_MHz_XTAL);
    save_item(NAME(m_pot_vstart));
    save_item(NAME(m_pot_vsize));
    save_item(NAME(m_pot_blnkcol));
    save_item(NAME(m_pot_hstart));
    save_item(NAME(m_pot_hsize));
    save_item(NAME(m_pot_cntl));
    save_item(NAME(m_pot_hintline));
    save_item(NAME(m_pot_int_enable));
    save_item(NAME(m_pot_int_status));
}

void solo1_asic_vid_device::device_reset()
{
	m_hsync_cb(false);
	m_vsync_cb(false);
    m_pot_int_enable = 0;
    m_pot_int_status = 0;
    m_vid_dmacntl = 0;
    m_vid_int_enable = 0;
    m_vid_int_status = 0;
    
	m_videobitmap.fill(0x0000);
}

uint32_t solo1_asic_vid_device::reg_pot_r(offs_t offset)
{
    // TODO: split this out into multiple handlers! using a giant switch statement for this is just ugly
    LOGMASKED(LOG_READS, "potUnit: read 9%03x\n", offset * 4);
    switch(offset * 4)
    {
    case 0x080: // POT_VSTART (R/W)
        return m_pot_vstart & 2047;
    case 0x084: // POT_VSIZE (R/W)
        return m_pot_vsize & 2047;
    case 0x088: // POT_BLNKCOL (R/W)
        return m_pot_blnkcol;
    case 0x08c: // POT_HSTART (R/W)
        return m_pot_hstart & 2047;
    case 0x090: // POT_HSIZE (R/W)
        return m_pot_hsize & 2047;
    case 0x094: // POT_CNTL (R/W)
        return m_pot_cntl;
    case 0x098: // POT_HINTLINE (R/W)
        return m_pot_hintline;
    case 0x09c: // POT_INTEN (R/Set)
        return m_pot_int_enable;
    case 0x0a0: // POT_INTSTAT (R/Set)
        return m_pot_int_status & m_pot_int_enable;
    case 0x0a4: // POT_INTEN (Clear)
        break;
    case 0x0a8: // POT_INTSTAT (Clear)
        break;
    case 0x0ac: // POT_CLINE (R/W)
        return m_pot_cline & 2047;
    default:
        logerror("Attempted read from reserved register 9%03x!\n", offset * 4);
        break;
    }
    return 0;
}

void solo1_asic_vid_device::reg_pot_w(offs_t offset, uint32_t data)
{
    // TODO: split this out into multiple handlers! using a giant switch statement for this is just ugly
    LOGMASKED(LOG_WRITES, "potUnit: write %08x to 9%03x\n", data, offset*4);
    switch(offset * 4)
    {
    case 0x080: // POT_VSTART (R/W)
        m_pot_vstart = data & 2047;
        if(m_pot_vstart + m_pot_vsize > SOLO1_NTSC_HEIGHT)
        {
            popmessage("Vertical window size is OUT OF RANGE!");
        }
        m_screen->set_visible_area(m_pot_hstart, m_pot_hstart + m_pot_hsize, m_pot_vstart, m_pot_vstart + m_pot_vsize);
        break;
    case 0x084: // POT_VSIZE (R/W)
        m_pot_vsize = data & 2047;
        if(m_pot_vstart + m_pot_vsize > SOLO1_NTSC_HEIGHT)
        {
            popmessage("Vertical window size is OUT OF RANGE!");
        }
        m_screen->set_visible_area(m_pot_hstart, m_pot_hstart + m_pot_hsize, m_pot_vstart, m_pot_vstart + m_pot_vsize);
        break;
    case 0x088: // POT_BLNKCOL (R/W)
        m_pot_blnkcol = data;
        break;
    case 0x08c: // POT_HSTART (R/W)
        m_pot_hstart = data & 2047;
        if(m_pot_hstart + m_pot_hsize > SOLO1_NTSC_WIDTH)
        {
            popmessage("Horizontal window is OUT OF RANGE!");
        }
        m_screen->set_visible_area(m_pot_hstart, m_pot_hstart+m_pot_hsize, m_pot_vstart, m_pot_vstart+m_pot_vsize);
        break;
    case 0x090: // POT_HSIZE (R/W)
        m_pot_hsize = data & 2047;
        if(m_pot_hstart + m_pot_hsize > SOLO1_NTSC_WIDTH)
        {
            popmessage("Horizontal window is OUT OF RANGE!");
        }
        m_screen->set_visible_area(m_pot_hstart, m_pot_hstart+m_pot_hsize, m_pot_vstart, m_pot_vstart+m_pot_vsize);
        break;
    case 0x094: // POT_CNTL (R/W)
        m_pot_cntl = data;
        if(m_pot_cntl & POT_CNTL_USE_GFXUNIT)
        {
            popmessage("gfxUnit source NOT IMPLEMENTED!");
        }
        break;
    case 0x098: // POT_HINTLINE (R/W)
        m_pot_hintline = data & 2047;
        break;
    case 0x09c: // POT_INTEN (R/Set)
        m_pot_int_enable |= data; // TODO: is this correct behavior?
        break;
    case 0x0a0: // POT_INTSTAT (R/Set)
        m_pot_int_status |= data; // TODO: is this correct behavior?
        break;
    case 0x0a4: // POT_INTEN (Clear)
        m_pot_int_enable &= ~data; // TODO: is this correct behavior?
        break;
    case 0x0a8: // POT_INTSTAT (Clear)
        m_pot_int_status &= ~data; // TODO: is this correct behavior?
        break;
    case 0x0ac: // POT_CLINE (R/W)
        m_pot_cline = data;
        break;
    default:
        logerror("Attempted write (%08x) to reserved register 9%03x!\n", data, offset*4);
        break;
    }
}

uint32_t solo1_asic_vid_device::reg_dve_r(offs_t offset)
{
    // TODO: split this out into multiple handlers! using a giant switch statement for this is just ugly
    LOGMASKED(LOG_READS, "dveUnit: read 7%03x\n", offset * 4);
    switch(offset * 4)
    {
    case 0x000: // DVE_CNTL (R/W)
        return m_dve_cntl;
    case 0x004: // DVE_CNFG (R/W)
        return m_dve_cnfg;
    case 0x008: // DVE_DBDATA (R/W)
        return m_dve_dbdata;
    case 0x00c: // DVE_DBEN (R/W)
        return m_dve_dben;
    case 0x010: // DVE_DTST (R/W)
        return m_dve_dtst;
    case 0x014: // DVE_RDFIELD (R)
        // TODO: return proper value
        return m_dve_rdfield;
    case 0x018: // DVE_RDPHASE (R)
        // TODO: return proper value
        return m_dve_rdphase;
    case 0x01c: // DVE_FILTCNTL (R/W)
        return m_dve_filtcntl;
    default:
        logerror("Attempted read from reserved register 7%03x!\n", offset * 4);
        break;
    }
    return 0;
}

void solo1_asic_vid_device::reg_dve_w(offs_t offset, uint32_t data)
{
    // TODO: split this out into multiple handlers! using a giant switch statement for this is just ugly
    LOGMASKED(LOG_WRITES, "dveUnit: write %08x to 7%03x\n", data, offset * 4);
    switch(offset * 4)
    {
    case 0x000: // DVE_CNTL (R/W)
        m_dve_cntl = data;
        break;
    case 0x004: // DVE_CNFG (R/W)
        m_dve_cnfg = data;
        break;
    case 0x008: // DVE_DBDATA (R/W)
        m_dve_dbdata = data;
        break;
    case 0x00c: // DVE_DBEN (R/W)
        m_dve_dben = data;
        break;
    case 0x010: // DVE_DTST (R/W)
        m_dve_dtst = data;
        break;
    case 0x014: // DVE_RDFIELD (R)
        logerror("Attempted write (%08x) to reserved register 7014 (DVE_RDFIELD)!\n", data);
        break;
    case 0x018: // DVE_RDPHASE (R)
        logerror("Attempted write (%08x) to reserved register 7018 (DVE_RDPHASE)!\n", data);
        break;
    case 0x01c: // DVE_FILTCNTL (R/W)
        m_dve_filtcntl = data;
        break;
    default:
        logerror("Attempted write (%08x) to reserved register 7%03x!\n", data, offset * 4);
        break;
    }
}

uint32_t solo1_asic_vid_device::reg_vid_r(offs_t offset)
{
    // TODO: split this out into multiple handlers! using a giant switch statement for this is just ugly
    LOGMASKED(LOG_READS, "vidUnit: read 3%03x\n", offset * 4);
    switch(offset * 4)
    {
    case 0x000: // VID_CSTART (R/W)
        return m_vid_cstart;
    case 0x004: // VID_CSIZE (R/W)
        return m_vid_csize;
    case 0x008: // VID_CCNT (R/W)
        return m_vid_ccount;
    case 0x00c: // VID_NSTART (R/W)
        return m_vid_nstart;
    case 0x010: // VID_NSIZE (R/W)
        return m_vid_nsize;
    case 0x014: // VID_DMACNTL (R/W)
        return m_vid_dmacntl;
    case 0x038: // VID_INTSTAT (R/Set)
        return m_vid_int_status & m_vid_int_enable;
    case 0x03c: // VID_INTEN (R/Set)
        return m_vid_int_enable;
    case 0x040: // VID_VDATA (R/W)
        return m_vid_vdata;
    case 0x138: // VID_INTSTAT (Clear)
        break;
    case 0x13c: // VID_INTEN (Clear)
        break;
    default:
        logerror("Attempted read from reserved register 3%03x!\n", offset * 4);
        break;
    }
    return 0;
}

void solo1_asic_vid_device::reg_vid_w(offs_t offset, uint32_t data)
{
    // TODO: split this out into multiple handlers! using a giant switch statement for this is just ugly
    LOGMASKED(LOG_WRITES, "vidUnit: write %08x to 3%03x\n", data, offset * 4);
    switch(offset * 4)
    {
    case 0x000: // VID_CSTART (R/W)
        logerror("Attempted write to read-only register 3000 (VID_CSTART)\n");
        break;
    case 0x004: // VID_CSIZE (R/W)
        logerror("Attempted write to read-only register 3004 (VID_CSIZE)\n");
        break;
    case 0x008: // VID_CCNT (R/W)
        logerror("Attempted write to read-only register 3008 (VID_CCNT)\n");
        break;
    case 0x00c: // VID_NSTART (R/W)
        m_vid_nstart = data;
        break;
    case 0x010: // VID_NSIZE (R/W)
        m_vid_nsize = data;
        break;
    case 0x014: // VID_DMACNTL (R/W)
        m_vid_dmacntl = data;
        break;
    case 0x038: // VID_INTSTAT (R/Set)
        m_vid_int_status |= data; // TODO: is this correct behavior?
        break;
    case 0x03c: // VID_INTEN (R/Set)
        m_vid_int_enable |= data; // TODO: is this correct behavior?
        break;
    case 0x040: // VID_VDATA (R/W)
        m_vid_vdata = data;
        break;
    case 0x138: // VID_INTSTAT (Clear)
        m_vid_int_status &= ~data; // TODO: is this correct behavior?
        break;
    case 0x13c: // VID_INTEN (Clear)
        m_vid_int_enable &= ~data; // TODO: is this correct behavior?
        break;
    default:
        logerror("Attempted write (%08x) to reserved register 3%03x!\n", data, offset * 4);
        break;
    }
}

uint32_t solo1_asic_vid_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
    /*
    m_videotex->set_bitmap(m_videobitmap, m_videobitmap.cliprect(), TEXFORMAT_YUY16);

    // reset the screen contents
    screen.container().empty();

    // add the video texture
    rgb_t videocolor = 0xffffffff; // Fully visible, white
    if ((m_pot_cntl&POT_CNTL_ENABLE_OUTPUTS) == 0)
        videocolor = 0xff000000; // Blank the texture's RGB of the texture
    m_screen->container().add_quad(0.0f, 0.0f, 1.0f, 1.0f, videocolor, m_videotex, PRIMFLAG_BLENDMODE(BLENDMODE_NONE) | PRIMFLAG_SCREENTEX(1));
    */
    return 0;
}

bool solo1_asic_vid_device::is_even_field()
{
    return false;
}
*/