// license:BSD-3-Clause
// copyright-holders:FairPlay137

/***********************************************************************************************

    solo1_asic_vid.cpp

    WebTV Networks Inc. SOLO1 ASIC (Video)

    This ASIC controls most of the I/O on the 2nd generation WebTV hardware.

    This implementation is based off of both the archived technical specifications, as well as
    the various reverse-engineering efforts of the WebTV community.

    The technical specifications that this implementation is based on can be found here:
    http://wiki.webtv.zone/misc/SOLO1/SOLO1_ASIC_Spec.pdf

    NOTE: This functionality is currently being handled within the main solo1_asic device. This
    device may serve as the video handler in the future once solo1_asic gets too complex, but
    for now this device is UNUSED.

************************************************************************************************/

/*
#ifndef MAME_MACHINE_SOLO1_ASIC_VID_H
#define MAME_MACHINE_SOLO1_ASIC_VID_H

#pragma once

#include "cpu/mips/mips3.h"
#include "solo1_asic.h"

class solo1_asic_vid_device : public device_t, public device_video_interface
{
public:
	// construction/destruction
	solo1_asic_vid_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
    
    uint32_t reg_vid_r(offs_t offset);
    void reg_vid_w(offs_t offset, uint32_t data);
    
    uint32_t reg_pot_r(offs_t offset);
    void reg_pot_w(offs_t offset, uint32_t data);
    
    uint32_t reg_dve_r(offs_t offset);
    void reg_dve_w(offs_t offset, uint32_t data);
    
	template <typename T> void set_hostcpu(T &&tag) { m_hostcpu.set_tag(std::forward<T>(tag)); }
    
    uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
    
    auto hsync_callback() { return m_hsync_cb.bind(); }
    auto vsync_callback() { return m_vsync_cb.bind(); }



    bool is_even_field();
    
    uint32_t m_pot_hintline; // Line where the interrupt happened
    uint32_t m_pot_int_enable;
    uint32_t m_pot_int_status;

protected:
    virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
    required_device<mips3_device> m_hostcpu;
	required_device<screen_device> m_screen;
    
    void fillbitmap_yuy16(bitmap_yuy16 &bitmap, uint8_t yval, uint8_t cr, uint8_t cb);
    uint16_t screen_height;

    uint32_t m_pot_vstart; // Vertical starting line
    uint32_t m_pot_vsize; // Vertical size
    uint32_t m_pot_blnkcol; // Blanking color
    uint32_t m_pot_hstart; // Horizontal starting pixel
    uint32_t m_pot_hsize; // Horizontal size
    uint32_t m_pot_cntl;
    uint32_t m_pot_cline; // Current line

    uint32_t m_vid_cstart;
    uint32_t m_vid_csize;
    uint32_t m_vid_ccount;
    uint32_t m_vid_nstart; // The address of the next video DMA
    uint32_t m_vid_nsize; // The number of bytes to be transferred for the next DMA
    uint32_t m_vid_dmacntl;
    uint32_t m_vid_int_status;
    uint32_t m_vid_int_enable;
    uint32_t m_vid_vdata;

    uint32_t m_dve_cntl;
    uint32_t m_dve_cnfg;
    uint32_t m_dve_dbdata;
    uint32_t m_dve_dben;
    uint32_t m_dve_dtst;
    uint32_t m_dve_rdfield; // read-only
    uint32_t m_dve_rdphase; // read-only
    uint32_t m_dve_filtcntl;

    uint32_t m_intstat;
    
	bitmap_yuy16        m_videobitmap;
	render_texture *    m_videotex;             // texture for the video
	palette_t *         m_videopalette;         // palette for the video
    
	devcb_write_line   m_hsync_cb;
	devcb_write_line   m_vsync_cb;

    TIMER_CALLBACK_MEMBER(vsync_on);
    TIMER_CALLBACK_MEMBER(vsync_off);
};

DECLARE_DEVICE_TYPE(SOLO1_ASIC_VID, solo1_asic_vid_device)

#endif*/